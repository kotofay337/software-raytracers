// ============================================================================
//  Tiny Ray Tracer — C++ / Win32 (MSVC 2022, без сторонних библиотек)
//  ---------------------------------------------------------------------------
//  Многопоточность: свой постоянный пул потоков (ThreadPool), воркеры
//  создаются один раз и переиспользуются; строки кадра раздаются
//  с шагом nThreads. Трассировка лучей — без изменений по сравнению
//  с исходным JavaFX-примером.
//
//  Сборка (x64 Native Tools Command Prompt for VS 2022):
//     cl /O2 /std:c++17 /EHsc /utf-8 tiny_ray_tracer.cpp /link user32.lib gdi32.lib
// ============================================================================

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <windows.h>
#include <windowsx.h>

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// ============================== КОНСТАНТЫ ===================================
static constexpr int    WIDTH = 640;
static constexpr int    HEIGHT = 400;
static constexpr int    SCALE = 1;
static constexpr int    MAX_DEPTH = 6;
static constexpr double PI = 3.14159265358979323846;
static constexpr double FOV = PI / 2.0;
static constexpr double MOUSE_SPHERE_Z = -16.0;
static constexpr int    MOUSE_SPHERE_IDX = 0;
static constexpr int    FPS_WINDOW = 30;

// tg(FOV/2) = tg(PI/4) = 1
static constexpr double TAN_HALF_FOV = 1.0;
static constexpr double ASPECT = (double)WIDTH / (double)HEIGHT;

// ================================ ВЕКТОР ====================================
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 add(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 sub(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 mul(double s)      const { return Vec3(x * s, y * s, z * s); }
    Vec3 neg()              const { return Vec3(-x, -y, -z); }

    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    double norm()           const { return std::sqrt(x * x + y * y + z * z); }

    Vec3 normalize() const {
        double n = std::sqrt(x * x + y * y + z * z);
        return n == 0.0 ? Vec3() : Vec3(x / n, y / n, z / n);
    }
};

static Vec3 reflect(const Vec3& i, const Vec3& n) {
    double d = 2.0 * (i.x * n.x + i.y * n.y + i.z * n.z);
    return Vec3(i.x - n.x * d, i.y - n.y * d, i.z - n.z * d);
}

static Vec3 refract(const Vec3& i, const Vec3& n, double eta) {
    double cosi = -std::max(-1.0, std::min(1.0, i.dot(n)));
    double etai = 1.0, etat = eta;
    Vec3   nn = n;
    if (cosi < 0) { cosi = -cosi; std::swap(etai, etat); nn = n.neg(); }
    double r = etai / etat;
    double k = 1.0 - r * r * (1.0 - cosi * cosi);
    if (k < 0) return Vec3();
    double s = r * cosi - std::sqrt(k);
    return Vec3(i.x * r + nn.x * s, i.y * r + nn.y * s, i.z * r + nn.z * s);
}

// ============================== МАТЕРИАЛ ====================================
struct Material {
    double refractiveIndex;
    double albedo[4];
    Vec3   diffuseColor;
    double specularExponent;

    Material()
        : refractiveIndex(1.0),
        albedo{ 1.0, 0.0, 0.0, 0.0 },
        diffuseColor(), specularExponent(0.0) {
    }

    Material(double ri, double a0, double a1, double a2, double a3,
        const Vec3& col, double spec)
        : refractiveIndex(ri),
        albedo{ a0, a1, a2, a3 },
        diffuseColor(col), specularExponent(spec) {
    }
};

// =============================== СФЕРА ======================================
struct Sphere {
    Vec3            center;
    double          radius;
    double          radius2;
    const Material* material;

    Sphere(const Vec3& c, double r, const Material* m)
        : center(c), radius(r), radius2(r* r), material(m) {
    }

    double rayIntersect(const Vec3& orig, const Vec3& dir) const {
        double lx = center.x - orig.x;
        double ly = center.y - orig.y;
        double lz = center.z - orig.z;
        double tca = lx * dir.x + ly * dir.y + lz * dir.z;
        double d2 = lx * lx + ly * ly + lz * lz - tca * tca;
        if (d2 > radius2) return -1.0;
        double thc = std::sqrt(radius2 - d2);
        double t0 = tca - thc;
        double t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return -1.0;
        return t0;
    }

    bool hitInRange(const Vec3& orig, const Vec3& dir, double maxT) const {
        double lx = center.x - orig.x;
        double ly = center.y - orig.y;
        double lz = center.z - orig.z;
        double tca = lx * dir.x + ly * dir.y + lz * dir.z;
        double d2 = lx * lx + ly * ly + lz * lz - tca * tca;
        if (d2 > radius2) return false;
        double thc = std::sqrt(radius2 - d2);
        double t0 = tca - thc;
        double t1 = tca + thc;
        if (t0 > 1e-3 && t0 < maxT) return true;
        if (t1 > 1e-3 && t1 < maxT) return true;
        return false;
    }
};

// ============================== СВЕТ И HIT ==================================
struct Light {
    Vec3   position;
    double intensity;
    Light(const Vec3& p, double i) : position(p), intensity(i) {}
};

struct Hit {
    Vec3            point;
    Vec3            normal;
    const Material* material = nullptr;
};

// ================================ СЦЕНА =====================================
static std::vector<Sphere> g_spheres;
static std::vector<Light>  g_lights;
static std::vector<Vec3>   g_basePositions;
static std::vector<Vec3>   g_rayDirs;

static const Vec3 ORIGIN(0.0, 0.0, 0.0);

static Material FLOOR_WHITE(1.0, 1.0, 0.0, 0.0, 0.0, Vec3(0.30, 0.30, 0.30), 0.0);
static Material FLOOR_ORANGE(1.0, 1.0, 0.0, 0.0, 0.0, Vec3(0.30, 0.21, 0.09), 0.0);

// ====================== ПЕРЕСЕЧЕНИЕ СО СЦЕНОЙ ===============================
static bool sceneIntersect(const Vec3& orig, const Vec3& dir, Hit& hit) {
    double bestT = 1e300;
    const Sphere* bestS = nullptr;

    const int n = (int)g_spheres.size();
    for (int i = 0; i < n; ++i) {
        const Sphere& s = g_spheres[i];
        double t = s.rayIntersect(orig, dir);
        if (t > 0.0 && t < bestT) {
            bestT = t;
            bestS = &s;
        }
    }

    // Пол
    double floorT = 1e300;
    double floorPx = 0.0, floorPz = 0.0;
    if (std::fabs(dir.y) > 1e-3) {
        double d = -(orig.y + 4.0) / dir.y;
        if (d > 0.0) {
            double px = orig.x + dir.x * d;
            double pz = orig.z + dir.z * d;
            if (std::fabs(px) < 10.0 && pz < -10.0 && pz > -30.0 && d < bestT) {
                floorT = d; floorPx = px; floorPz = pz;
            }
        }
    }

    if (floorT < bestT && floorT < 1000.0) {
        hit.point = Vec3(orig.x + dir.x * floorT,
            orig.y + dir.y * floorT,
            orig.z + dir.z * floorT);
        hit.normal = Vec3(0.0, 1.0, 0.0);
        int cell = (int)(0.5 * floorPx + 1000.0) + (int)(0.5 * floorPz);
        hit.material = (cell & 1) != 0 ? &FLOOR_WHITE : &FLOOR_ORANGE;
        return true;
    }

    if (bestS != nullptr && bestT < 1000.0) {
        double px = orig.x + dir.x * bestT;
        double py = orig.y + dir.y * bestT;
        double pz = orig.z + dir.z * bestT;
        hit.point = Vec3(px, py, pz);

        double nx = px - bestS->center.x;
        double ny = py - bestS->center.y;
        double nz = pz - bestS->center.z;
        double invN = 1.0 / std::sqrt(nx * nx + ny * ny + nz * nz);
        hit.normal = Vec3(nx * invN, ny * invN, nz * invN);
        hit.material = bestS->material;
        return true;
    }

    return false;
}

// ============================ ТЕНЕВОЙ ЛУЧ ===================================
static bool isShadowed(const Vec3& orig, const Vec3& dir, double maxDist) {
    const int n = (int)g_spheres.size();
    for (int i = 0; i < n; ++i) {
        if (g_spheres[i].hitInRange(orig, dir, maxDist)) return true;
    }
    if (std::fabs(dir.y) > 1e-3) {
        double d = -(orig.y + 4.0) / dir.y;
        if (d > 1e-3 && d < maxDist) {
            double px = orig.x + dir.x * d;
            double pz = orig.z + dir.z * d;
            if (std::fabs(px) < 10.0 && pz < -10.0 && pz > -30.0) return true;
        }
    }
    return false;
}

// ====================== ГЛАВНЫЙ РЕКУРСИВНЫЙ ТРАССИРОВЩИК ====================
static Vec3 castRay(const Vec3& orig, const Vec3& dir, int depth) {
    if (depth > MAX_DEPTH) {
        return Vec3(0.2, 0.7, 0.8);
    }

    Hit hit;
    if (!sceneIntersect(orig, dir, hit)) {
        return Vec3(0.2, 0.7, 0.8);
    }

    const Vec3& point = hit.point;
    const Vec3& N = hit.normal;
    const Material* mat = hit.material;

    // ---------- Отражение ----------
    Vec3 reflectColor;
    double wRefl = mat->albedo[2];
    if (wRefl > 0.0) {
        Vec3 reflectDir = reflect(dir, N).normalize();
        double eps = reflectDir.dot(N) < 0.0 ? -1e-3 : 1e-3;
        Vec3 reflectOrig(point.x + N.x * eps,
            point.y + N.y * eps,
            point.z + N.z * eps);
        reflectColor = castRay(reflectOrig, reflectDir, depth + 1);
    }

    // ---------- Преломление ----------
    Vec3 refractColor;
    double wRefr = mat->albedo[3];
    if (wRefr > 0.0) {
        Vec3 refractDir = refract(dir, N, mat->refractiveIndex).normalize();
        double eps = refractDir.dot(N) < 0.0 ? -1e-3 : 1e-3;
        Vec3 refractOrig(point.x + N.x * eps,
            point.y + N.y * eps,
            point.z + N.z * eps);
        refractColor = castRay(refractOrig, refractDir, depth + 1);
    }

    // ---------- Прямое освещение и тени ----------
    double diffInt = 0.0;
    double specInt = 0.0;
    const int lightCount = (int)g_lights.size();
    for (int li = 0; li < lightCount; ++li) {
        const Light& light = g_lights[li];
        double lx = light.position.x - point.x;
        double ly = light.position.y - point.y;
        double lz = light.position.z - point.z;
        double lightDistSq = lx * lx + ly * ly + lz * lz;
        double lightDist = std::sqrt(lightDistSq);
        double invDist = 1.0 / lightDist;
        double dirx = lx * invDist, diry = ly * invDist, dirz = lz * invDist;

        double ndl = dirx * N.x + diry * N.y + dirz * N.z;
        double eps = ndl < 0.0 ? -1e-3 : 1e-3;
        Vec3 shadowOrig(point.x + N.x * eps,
            point.y + N.y * eps,
            point.z + N.z * eps);

        if (isShadowed(shadowOrig, Vec3(dirx, diry, dirz), lightDist)) continue;

        if (ndl > 0.0) diffInt += light.intensity * ndl;

        if (mat->specularExponent > 0.0) {
            double ldn = -ndl;
            double rx = -dirx - 2.0 * ldn * N.x;
            double ry = -diry - 2.0 * ldn * N.y;
            double rz = -dirz - 2.0 * ldn * N.z;
            double rd = -(rx * dir.x + ry * dir.y + rz * dir.z);
            if (rd > 0.0)
                specInt += std::pow(rd, mat->specularExponent) * light.intensity;
        }
    }

    const double a0 = mat->albedo[0], a1 = mat->albedo[1];
    const double dcx = mat->diffuseColor.x;
    const double dcy = mat->diffuseColor.y;
    const double dcz = mat->diffuseColor.z;

    double r = dcx * diffInt * a0 + specInt * a1 + reflectColor.x * wRefl + refractColor.x * wRefr;
    double g = dcy * diffInt * a0 + specInt * a1 + reflectColor.y * wRefl + refractColor.y * wRefr;
    double b = dcz * diffInt * a0 + specInt * a1 + reflectColor.z * wRefl + refractColor.z * wRefr;

    return Vec3(r, g, b);
}

// ==================== ПРЕДВЫЧИСЛЕНИЕ НАПРАВЛЕНИЙ ЛУЧЕЙ ======================
static void precomputeRayDirs() {
    g_rayDirs.resize((size_t)WIDTH * HEIGHT);
    for (int j = 0; j < HEIGHT; ++j) {
        double y = -(2.0 * (j + 0.5) / HEIGHT - 1.0) * TAN_HALF_FOV;
        const int row = j * WIDTH;
        for (int i = 0; i < WIDTH; ++i) {
            double x = (2.0 * (i + 0.5) / WIDTH - 1.0) * TAN_HALF_FOV * ASPECT;
            double inv = 1.0 / std::sqrt(x * x + y * y + 1.0);
            g_rayDirs[(size_t)row + i] = Vec3(x * inv, y * inv, -inv);
        }
    }
}

// ============================== ПУЛ ПОТОКОВ =================================
// Синхронный пул: N воркеров создаются один раз и переиспользуются.
// main кладёт задачу, «поколение» gen_ увеличивается, воркеры просыпаются,
// каждый выполняет task(idx, nThreads); main ждёт done_ == N.
// ============================================================================
class ThreadPool {
public:
    using Task = std::function<void(int /*idx*/, int /*nThreads*/)>;

    explicit ThreadPool(int n)
        : n_(n), gen_(0), done_(0), stop_(false) {
        threads_.reserve(n_);
        for (int i = 0; i < n_; ++i)
            threads_.emplace_back([this, i]() { worker(i); });
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            stop_ = true;
            ++gen_;
        }
        cv_.notify_all();
        for (auto& t : threads_) t.join();
    }

    void run(Task task) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            task_ = std::move(task);
            done_ = 0;
            ++gen_;
        }
        cv_.notify_all();

        std::unique_lock<std::mutex> lk(mtx_);
        cvDone_.wait(lk, [this]() { return done_ == n_; });
    }

private:
    void worker(int idx) {
        int seen = 0;
        for (;;) {
            Task local;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait(lk, [this, &seen]() { return gen_ != seen; });
                seen = gen_;
                if (stop_) return;
                local = task_;
            }
            local(idx, n_);

            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (++done_ == n_) cvDone_.notify_one();
            }
        }
    }

    int                       n_;
    int                       gen_;
    int                       done_;
    bool                      stop_;
    Task                      task_;
    std::mutex                mtx_;
    std::condition_variable   cv_;
    std::condition_variable   cvDone_;
    std::vector<std::thread>  threads_;
};

// ============================ ГЛОБАЛЬНОЕ СОСТОЯНИЕ ==========================
static HWND        g_hwnd = nullptr;
static HDC         g_memDC = nullptr;
static HBITMAP     g_hBitmap = nullptr;
static uint32_t* g_dibBits = nullptr;
static HFONT       g_font = nullptr;
static bool        g_dragging = false;
static ThreadPool* g_pool = nullptr;

// ============================================================================
// РЕНДЕР КАДРА. Воркер idx обрабатывает строки idx, idx+N, idx+2N, ...
// ============================================================================
static void renderToBuffer(uint32_t* pixels) {
    const Vec3* dirs = g_rayDirs.data();

    g_pool->run([pixels, dirs](int idx, int nThreads) {
        for (int j = idx; j < HEIGHT; j += nThreads) {
            const int row = j * WIDTH;
            for (int i = 0; i < WIDTH; ++i) {
                const Vec3& dir = dirs[(size_t)row + i];
                Vec3 c = castRay(ORIGIN, dir, 0);

                double mx = std::max(c.x, std::max(c.y, c.z));
                double scale = mx > 1.0 ? 1.0 / mx : 1.0;

                int r = (int)(255.0 * std::max(0.0, std::min(1.0, c.x * scale)));
                int g = (int)(255.0 * std::max(0.0, std::min(1.0, c.y * scale)));
                int b = (int)(255.0 * std::max(0.0, std::min(1.0, c.z * scale)));

                pixels[(size_t)row + i] = 0xFF000000u
                    | ((uint32_t)r << 16)
                    | ((uint32_t)g << 8)
                    | (uint32_t)b;
            }
        }
        });
}

// ======================= МЫШЬ → МИРОВЫЕ КООРДИНАТЫ ==========================
static Vec3 screenToWorld(double mx, double my) {
    double nx = mx / (double)(WIDTH * SCALE);
    double ny = my / (double)(HEIGHT * SCALE);
    double tx = (2.0 * nx - 1.0) * TAN_HALF_FOV * ASPECT;
    double ty = -(2.0 * ny - 1.0) * TAN_HALF_FOV;
    double depth = -MOUSE_SPHERE_Z;
    return Vec3(tx * depth, ty * depth, MOUSE_SPHERE_Z);
}

// ============================== WNDPROC =====================================
static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_LBUTTONDOWN: {
        g_dragging = true;
        int mx = GET_X_LPARAM(lp);
        int my = GET_Y_LPARAM(lp);
        g_spheres[MOUSE_SPHERE_IDX].center = screenToWorld((double)mx, (double)my);
        return 0;
    }
    case WM_MOUSEMOVE:
        if (g_dragging) {
            int mx = GET_X_LPARAM(lp);
            int my = GET_Y_LPARAM(lp);
            g_spheres[MOUSE_SPHERE_IDX].center = screenToWorld((double)mx, (double)my);
        }
        return 0;
    case WM_LBUTTONUP:
        g_dragging = false;
        return 0;

    case WM_ERASEBKGND:
        return 1;

    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        if (g_memDC)
            BitBlt(hdc, 0, 0, WIDTH * SCALE, HEIGHT * SCALE,
                g_memDC, 0, 0, SRCCOPY);
        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

// ============================== WINMAIN =====================================
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    // --------------------------- МАТЕРИАЛЫ ---------------------------------
    Material ivory(1.0, 0.6, 0.3, 0.1, 0.0, Vec3(0.4, 0.4, 0.3), 50.0);
    Material glass(1.5, 0.0, 0.5, 0.1, 0.8, Vec3(0.6, 0.7, 0.8), 125.0);
    Material redRubber(1.0, 0.9, 0.1, 0.0, 0.0, Vec3(0.3, 0.1, 0.1), 10.0);
    Material mirror(1.0, 0.0, 10.0, 0.8, 0.0, Vec3(1.0, 1.0, 1.0), 1425.0);

    // --------------------------- СФЕРЫ -------------------------------------
    // Индекс 0 — та, которой управляет мышь.
    g_spheres.emplace_back(Vec3(7.0, 5.0, -18.0), 4.0, &mirror);
    g_spheres.emplace_back(Vec3(-3.0, 0.0, -16.0), 2.0, &ivory);
    g_spheres.emplace_back(Vec3(-1.0, -1.5, -12.0), 2.0, &glass);
    g_spheres.emplace_back(Vec3(1.5, -0.5, -18.0), 3.0, &redRubber);

    // --------------------------- ИСТОЧНИКИ ---------------------------------
    g_lights.emplace_back(Vec3(-20.0, 20.0, 20.0), 1.5);
    g_lights.emplace_back(Vec3(30.0, 50.0, -25.0), 1.8);
    g_lights.emplace_back(Vec3(30.0, 20.0, 30.0), 1.7);

    // --------------------------- БАЗОВЫЕ ПОЗИЦИИ ---------------------------
    g_basePositions.resize(g_spheres.size());
    for (size_t i = 0; i < g_spheres.size(); ++i)
        g_basePositions[i] = g_spheres[i].center;

    // --------------------------- ЛУЧИ КАМЕРЫ -------------------------------
    precomputeRayDirs();

    // --------------------------- ПУЛ ПОТОКОВ -------------------------------
    unsigned hw = std::thread::hardware_concurrency();
    int nThreads = (hw == 0) ? 1 : (int)hw;
    ThreadPool pool(nThreads);
    g_pool = &pool;

    // --------------------------- ОКНО --------------------------------------
    const DWORD style = (WS_OVERLAPPEDWINDOW & ~(WS_THICKFRAME | WS_MAXIMIZEBOX));

    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(wc);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;
    wc.lpszClassName = L"TinyRayTracerFXClass";
    RegisterClassExW(&wc);

    RECT rc = { 0, 0, WIDTH * SCALE, HEIGHT * SCALE };
    AdjustWindowRect(&rc, style, FALSE);

    g_hwnd = CreateWindowExW(
        0, L"TinyRayTracerFXClass",
        L"Tiny Ray Tracer — C++/Win32 (ThreadPool)",
        style,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right - rc.left, rc.bottom - rc.top,
        nullptr, nullptr, hInstance, nullptr);
    if (!g_hwnd) return 1;

    // --------------------------- DIB-СЕКЦИЯ --------------------------------
    HDC hdcScreen = GetDC(g_hwnd);

    BITMAPINFO bmi = {};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = WIDTH;
    bmi.bmiHeader.biHeight = -HEIGHT;    // top-down DIB
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;         // BGRA
    bmi.bmiHeader.biCompression = BI_RGB;

    void* bits = nullptr;
    g_hBitmap = CreateDIBSection(hdcScreen, &bmi, DIB_RGB_COLORS, &bits, nullptr, 0);
    g_dibBits = reinterpret_cast<uint32_t*>(bits);

    g_memDC = CreateCompatibleDC(hdcScreen);
    SelectObject(g_memDC, g_hBitmap);
    ReleaseDC(g_hwnd, hdcScreen);

    // --------------------------- ШРИФТ -------------------------------------
    g_font = CreateFontW(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, FF_MODERN, L"Consolas");

    ShowWindow(g_hwnd, nCmdShow);
    UpdateWindow(g_hwnd);

    // ========================================================================
    // ГЛАВНЫЙ ЦИКЛ
    // ========================================================================
    LARGE_INTEGER qpcFreqLi;
    QueryPerformanceFrequency(&qpcFreqLi);
    const double qpcFreq = (double)qpcFreqLi.QuadPart;

    long long frameTimes[FPS_WINDOW] = {};
    int    frameIndex = 0;
    int    framesFilled = 0;
    double fps = 0.0;
    long long t0 = 0;

    bool running = true;
    MSG  msg;

    while (running) {
        // ---- обработка сообщений Windows ----
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) { running = false; break; }
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (!running) break;

        // ---- текущее время ----
        LARGE_INTEGER nowLi;
        QueryPerformanceCounter(&nowLi);
        const long long now = nowLi.QuadPart;

        // ---- FPS (скользящее окно) ----
        frameTimes[frameIndex] = now;
        frameIndex = (frameIndex + 1) % FPS_WINDOW;
        if (framesFilled < FPS_WINDOW) ++framesFilled;

        if (framesFilled >= 2) {
            int oldestIdx = (frameIndex - framesFilled + FPS_WINDOW) % FPS_WINDOW;
            long long dt = now - frameTimes[oldestIdx];
            if (dt > 0)
                fps = (framesFilled - 1) * qpcFreq / (double)dt;
        }

        if (t0 == 0) t0 = now;
        double t = (double)(now - t0) / qpcFreq;

        // ---- анимация сфер ----
        for (size_t i = 0; i < g_spheres.size(); ++i) {
            if ((int)i == MOUSE_SPHERE_IDX) continue;
            const Vec3 base = g_basePositions[i];
            double fi = (double)i;
            double dx = std::sin(t * 0.8 + fi * 1.7) * (1.7 + fi * 0.25);
            double dy = std::sin(t * 1.1 + fi * 2.1) * 1.4;
            double dz = std::cos(t * 0.6 + fi * 1.3) * (1.4 + fi * 0.20);
            g_spheres[i].center = Vec3(base.x + dx, base.y + dy, base.z + dz);
        }

        // ---- рендер кадра (пул потоков) ----
        renderToBuffer(g_dibBits);

        // ---- текст FPS прямо в DIB ----
        char fpsText[64];
        std::snprintf(fpsText, sizeof(fpsText), "FPS: %.1f", fps);

        SetBkMode(g_memDC, TRANSPARENT);
        SetTextColor(g_memDC, RGB(255, 255, 255));
        HGDIOBJ oldFont = SelectObject(g_memDC, g_font);
        TextOutA(g_memDC, WIDTH * SCALE - 100, 8,
            fpsText, (int)std::strlen(fpsText));
        SelectObject(g_memDC, oldFont);

        // ---- вывод кадра на экран ----
        HDC hdc = GetDC(g_hwnd);
        BitBlt(hdc, 0, 0, WIDTH * SCALE, HEIGHT * SCALE,
            g_memDC, 0, 0, SRCCOPY);
        ReleaseDC(g_hwnd, hdc);
    }

    // --------------------------- ОЧИСТКА -----------------------------------
    // Сбрасываем указатель на пул и даём локальному pool уничтожиться
    // при выходе из WinMain. К этому моменту рабочие потоки простаивают
    // в cv_.wait() и к DIB не обращаются, так что порядок безопасен.
    g_pool = nullptr;

    if (g_font) { DeleteObject(g_font);   g_font = nullptr; }
    if (g_hBitmap) { DeleteObject(g_hBitmap); g_hBitmap = nullptr; }
    if (g_memDC) { DeleteDC(g_memDC);      g_memDC = nullptr; }

    return 0;
}