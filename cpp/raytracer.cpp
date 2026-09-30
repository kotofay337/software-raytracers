// ============================================================================
//  Tiny Ray Tracer — C++ / Win32 (MSVC 2022, без сторонних библиотек)
//  ---------------------------------------------------------------------------
//  УЧЕБНЫЙ ПОРТ JavaFX-примера "TinyRayTracerFX" + КУБ и ПИРАМИДА.
//
//  Что демонстрирует программа:
//    1. Классическая обратная трассировка лучей (Whitted-style):
//       из камеры выпускаются лучи, ищут пересечения со сферами,
//       треугольниками и шахматной плоскостью, затем рекурсивно
//       отражаются и преломляются.
//    2. Модель освещения: диффузная составляющая (Ламберт), зеркальный
//       блик (Фонг), тени через теневые лучи.
//    3. Материалы: рассеивающие, стекло, металлы — через вектор
//       albedo (вклад diffuse / specular / reflect / refract).
//    4. Полигональная геометрия: куб и пирамида как наборы
//       треугольников; пересечение луча и треугольника по
//       алгоритму Möller–Trumbore.
//    5. Анимация: сферы колеблются по синусоидам, полигональные
//       фигуры ещё и вращаются вокруг оси Y.
//    6. Многопоточный рендер: свой маленький пул потоков делит
//       кадр на строки между воркерами.
//    7. Нативный GUI на WinAPI: окно, DIB-секция как кадровый
//       буфер, обработка мыши, вывод FPS прямо в буфер.
//
//  Сборка (x64 Native Tools Command Prompt for VS 2022):
//     cl /O2 /std:c++17 /EHsc /utf-8 tiny_ray_tracer.cpp /link user32.lib gdi32.lib
//
//  Или в Visual Studio: "Empty Project (C++)", добавить этот файл,
//  выставить C++17 и собрать.
// ============================================================================

#define WIN32_LEAN_AND_MEAN   // не тянуть в windows.h лишние заголовки
#define NOMINMAX              // отключить макросы min/max из windows.h —
                              // иначе они конфликтуют со std::min/std::max

#include <windows.h>
#include <windowsx.h>         // макросы GET_X_LPARAM / GET_Y_LPARAM для мыши

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// ============================== КОНСТАНТЫ ===================================
// Разрешение кадрового буфера (в пикселях). Именно столько лучей мы
// выпускаем за кадр. 480*320 = 153 600 пикселей — компромисс между
// качеством картинки и скоростью. С учётом рекурсии и теневых лучей
// реальное число пересечений за кадр в разы больше.
static constexpr int    WIDTH = 640;
static constexpr int    HEIGHT = 480;

// Во сколько раз растягивать буфер при выводе на экран.
// SCALE = 1 значит 1 пиксель буфера = 1 физический пиксель окна.
static constexpr int    SCALE = 1;

// Максимальная глубина рекурсии для отражений/преломлений.
// Чем больше — тем больше "отражений в отражениях" видно, но
// стоимость растёт экспоненциально (каждый узел даёт 2 ветки).
static constexpr int    MAX_DEPTH = 6;

static constexpr double PI = 3.14159265358979323846;

// Угол обзора по вертикали. PI/2 = 90°.
static constexpr double FOV = PI / 2.5;

// Глубина, на которой «висит» сфера, управляемая мышью.
// Ось Z смотрит "в экран" (отрицательное Z = вглубь сцены),
// камера находится в начале координат.
static constexpr double MOUSE_SPHERE_Z = -16.0;

// Индекс сферы в векторе spheres, которой мы управляем мышью.
static constexpr int    MOUSE_SPHERE_IDX = 0;

// ================================ ВЕКТОР ====================================
// Минималистичный неизменяемый 3D-вектор. Все методы возвращают НОВЫЙ
// вектор, а не мутируют текущий — это делает код трассировки намного
// понятнее и безопаснее для многопоточного чтения.
struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // Покомпонентные операции — линейная алгебра "по определению".
    Vec3  add(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3  sub(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3  mul(double s)      const { return Vec3(x * s, y * s, z * s); }
    Vec3  neg()              const { return Vec3(-x, -y, -z); }

    // Скалярное произведение: a·b = |a||b|cos(угол).
    // Ключевая операция трассировки: проверка углов, проекции,
    // вычисление освещения.
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }

    // Евклидова длина: sqrt(x²+y²+z²).
    double norm() const { return std::sqrt(dot(*this)); }

    // ------------------------------------------------------------------
    // ВЕКТОРНОЕ ПРОИЗВЕДЕНИЕ. В отличие от dot, результат — вектор,
    // перпендикулярный обоим входным. |a×b| = |a||b|sin(угла), а
    // направление определяется правилом правой руки.
    //
    // Формула — раскрытие определителя 3x3:
    //     | i  j  k  |
    //     | ax ay az |
    //     | bx by bz |
    //
    // Нужно в двух ключевых местах:
    //  1) нормаль треугольника = cross(v1-v0, v2-v0);
    //  2) Möller–Trumbore — весь алгоритм построен на cross.
    // ------------------------------------------------------------------
    Vec3 cross(const Vec3& o) const {
        return Vec3(y * o.z - z * o.y,
            z * o.x - x * o.z,
            x * o.y - y * o.x);
    }

    // Приведение к единичной длине. Если длина ноль (нулевой вектор
    // или численный шум) — возвращаем ноль, чтобы избежать NaN.
    Vec3 normalize() const {
        double n = norm();
        return n == 0.0 ? Vec3() : mul(1.0 / n);
    }
};

// ----------------------------------------------------------------------------
// Зеркальное отражение луча i от поверхности с нормалью n (n — единичный):
//     r = i - 2 (i·n) n
// Идея: раскладываем i на нормальную и касательную составляющие,
// нормальная меняет знак, касательная остаётся.
// ----------------------------------------------------------------------------
static Vec3 reflect(const Vec3& i, const Vec3& n) {
    return i.sub(n.mul(2.0 * i.dot(n)));
}

// ----------------------------------------------------------------------------
// Преломление по закону Снеллиуса: n1 sin θ1 = n2 sin θ2.
// Параметры:
//     i   — падающий луч (единичный);
//     n   — нормаль поверхности (единичная);
//     eta — n2/n1 для перехода "воздух → среда".
//
// Если i·n < 0, луч входит в среду; иначе выходит, и мы
// разворачиваем нормаль и меняем местами n1 ↔ n2.
//
// Возвращаем единичный вектор преломлённого луча. При полном
// внутреннем отражении (k < 0) возвращаем нулевой вектор —
// трассировщик умножит его на albedo[3], и вклад обнулится.
// ----------------------------------------------------------------------------
static Vec3 refract(const Vec3& i, const Vec3& n, double eta) {
    double cosi = -std::max(-1.0, std::min(1.0, i.dot(n)));
    double etai = 1.0, etat = eta;
    Vec3   nn = n;

    if (cosi < 0) {
        // Луч "выходит" из среды: разворачиваем нормаль и меняем
        // местами показатели преломления.
        cosi = -cosi;
        std::swap(etai, etat);
        nn = n.neg();
    }

    double r = etai / etat;                    // n1/n2
    // sin²θ2 = r² sin²θ1. Если это > 1 — полное внутреннее отражение.
    double k = 1.0 - r * r * (1.0 - cosi * cosi);
    if (k < 0) return Vec3();

    // Векторная форма преломления.
    return i.mul(r).add(nn.mul(r * cosi - std::sqrt(k)));
}

// ============================== МАТЕРИАЛ ====================================
// Оптические свойства поверхности.
//
// refractiveIndex — показатель преломления (1.0 = воздух, 1.5 ≈ стекло).
//
// albedo[4] — четыре весовых коэффициента, сумма вкладов:
//     albedo[0] — диффузное освещение (рассеянный свет, Ламберт)
//     albedo[1] — зеркальный блик (Фонг)
//     albedo[2] — зеркальное отражение (reflection)
//     albedo[3] — преломление (refraction)
//
// Сумма не обязана быть 1 — можно делать "усиленные" блики
// (albedo[1] = 10 у зеркала — artistic license).
struct Material {
    double refractiveIndex;
    double albedo[4];
    Vec3   diffuseColor;        // собственный цвет материала
    double specularExponent;    // чем больше — тем острее блик

    // Пустой материал нужен для плоскости: заполняем только
    // пару полей (diffuseColor), остальное — дефолт.
    Material()
        : refractiveIndex(1.0),
        albedo{ 1.0, 0.0, 0.0, 0.0 },
        diffuseColor(),
        specularExponent(0.0) {
    }

    Material(double ri,
        double a0, double a1, double a2, double a3,
        const Vec3& col, double spec)
        : refractiveIndex(ri),
        albedo{ a0, a1, a2, a3 },
        diffuseColor(col),
        specularExponent(spec) {
    }
};

// =============================== СФЕРА ======================================
struct Sphere {
    Vec3     center;
    double   radius;
    Material material;

    Sphere(const Vec3& c, double r, const Material& m)
        : center(c), radius(r), material(m) {
    }

    // ------------------------------------------------------------------
    // Пересечение луча со сферой. Классический геометрический вывод:
    //
    //   Точка на луче:   P(t) = orig + t·dir,  |dir| = 1
    //   Уравнение сферы: |P(t) - C|² = r²
    //   Обозначим L = C - orig.
    //
    //   t² - 2 t (L·dir) + L·L - r² = 0
    //
    //   tca = L·dir  — проекция L на направление луча (координата
    //                  ближайшей к центру точки вдоль луча).
    //   d2 = L·L - tca² — квадрат расстояния от центра сферы
    //                     до прямой луча.
    //
    //   Если d2 > r² — прямая сферу не задевает.
    //   thc = sqrt(r² - d2) — половина длины хорды.
    //   t0 = tca - thc (ближнее), t1 = tca + thc (дальнее).
    //
    //   t0 < 0 — начало луча внутри сферы, берём t1.
    //   t1 < 0 — сфера полностью за нами — нет пересечения.
    // ------------------------------------------------------------------
    bool rayIntersect(const Vec3& orig, const Vec3& dir, double& outT) const {
        Vec3   L = center.sub(orig);
        double tca = L.dot(dir);
        double d2 = L.dot(L) - tca * tca;
        double r2 = radius * radius;
        if (d2 > r2) return false;
        double thc = std::sqrt(r2 - d2);
        double t0 = tca - thc;
        double t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        outT = t0;
        return true;
    }
};

// ============================== ТРЕУГОЛЬНИК =================================
// Универсальный примитив для полигональной геометрии: куб, пирамида,
// любой другой меш — всё это наборы треугольников.
//
// ----------------------------------------------------------------------------
// ПЕРЕСЕЧЕНИЕ ЛУЧА И ТРЕУГОЛЬНИКА — алгоритм Möller–Trumbore (1997).
//
// Идея: точка внутри треугольника выражается через БАРИЦЕНТРИЧЕСКИЕ
// КООРДИНАТЫ (u, v):
//
//     P(u, v) = v0 + u·(v1-v0) + v·(v2-v0),
//     при этом u ≥ 0, v ≥ 0, u + v ≤ 1.
//
// Подставляем P = orig + t·dir:
//
//     orig + t·dir = v0 + u·e1 + v·e2,   где e1 = v1-v0, e2 = v2-v0.
//
// Переносим v0 и обозначаем s = orig - v0:
//
//     t·dir - u·e1 - v·e2 = s.
//
// Это система 3x3 относительно (t, u, v). Möller–Trumbore решает
// её через правило Крамера, где определители считаются через
// смешанное произведение (a · (b × c)). Отсюда и появление cross.
//
// Схема:
//     h = dir × e2
//     a = e1 · h            — определитель системы
//     Если |a| < EPS — луч параллелен плоскости треугольника.
//     f = 1 / a
//     s = orig - v0
//     u = f · (s · h)                          → проверяем 0 ≤ u ≤ 1
//     q = s × e1
//     v = f · (dir · q)                        → проверяем 0 ≤ v, u+v ≤ 1
//     t = f · (e2 · q)                         → проверяем t > EPS
//
// Вся арифметика — только через векторные операции. Никаких
// обратных матриц, никаких подстановок в плоскость.
// ----------------------------------------------------------------------------
//
// Тонкости:
//   * EPS = 1e-8 — и для случая "луч параллелен треугольнику"
//     (|a| ≈ 0), и для отсечения пересечений "за камерой" (t < EPS).
//   * Нормаль = cross(e1, e2). Её направление зависит от порядка
//     вершин в треугольнике. Чтобы освещение работало одинаково
//     с обеих сторон (важно для ОТКРЫТЫХ мешей — например, у
//     пирамиды нет нижней крышки), разворачиваем нормаль навстречу
//     лучу. Это даёт ту же семантику, что и у сферы (нормаль
//     "наружу" от поверхности в сторону луча).
// ============================================================================
struct Triangle {
    Vec3     v0, v1, v2;
    Material material;

    bool rayIntersect(const Vec3& orig, const Vec3& dir,
        double& outT, Vec3& outNormal) const {
        constexpr double EPS = 1e-8;

        Vec3   edge1 = v1.sub(v0);       // e1
        Vec3   edge2 = v2.sub(v0);       // e2

        Vec3   h = dir.cross(edge2);     // h = dir × e2
        double a = edge1.dot(h);         // определитель системы

        // Луч параллелен плоскости треугольника — пересечения нет.
        if (std::fabs(a) < EPS) return false;

        double f = 1.0 / a;
        Vec3   s = orig.sub(v0);

        // Первая барицентрическая координата u.
        double u = f * s.dot(h);
        if (u < 0.0 || u > 1.0) return false;

        // Вторая барицентрическая координата v.
        Vec3   q = s.cross(edge1);
        double v = f * dir.dot(q);
        if (v < 0.0 || u + v > 1.0) return false;

        // Расстояние до точки пересечения вдоль луча.
        double t = f * edge2.dot(q);
        if (t < EPS) return false;       // пересечение "за камерой"

        outT = t;
        outNormal = edge1.cross(edge2).normalize();

        // Разворачиваем нормаль навстречу лучу, чтобы освещение
        // работало корректно и с "изнанки".
        if (outNormal.dot(dir) > 0.0) outNormal = outNormal.neg();
        return true;
    }
};

// ============================== СВЕТ И HIT ==================================
// Точечный источник: белый, регулируется интенсивностью.
struct Light { Vec3 position; double intensity; };

// Результат ближайшего попадания луча в сцену.
struct Hit {
    Vec3     point;
    Vec3     normal;
    Material material;
};

// ================================ СЦЕНА =====================================
// Все объекты в одном месте. basePositions — "исходные" позиции сфер,
// от которых считается синусоидальная анимация.
struct Scene {
    std::vector<Sphere>   spheres;
    std::vector<Triangle> triangles;      // куб, пирамида и т.п.
    std::vector<Vec3>     basePositions;  // базы анимации сфер
    std::vector<Light>    lights;
};
static Scene g_scene;

// ----------------------------------------------------------------------------
// БЛИЖАЙШЕЕ ПЕРЕСЕЧЕНИЕ ЛУЧА СО СЦЕНОЙ.
//
// Проходим по всем объектам, держим "текущее лучшее" расстояние
// и обновляем hit, если нашли что-то ближе. Порядок обхода не важен,
// главное — корректное сравнение t.
// ----------------------------------------------------------------------------
static bool sceneIntersect(const Vec3& orig, const Vec3& dir, Hit& hit) {
    double bestDist = 1e300;

    // ---- Сферы ----
    for (const Sphere& s : g_scene.spheres) {
        double t;
        if (s.rayIntersect(orig, dir, t) && t < bestDist) {
            bestDist = t;
            hit.point = orig.add(dir.mul(t));
            // Нормаль сферы = (P - C) / r. normalize() сам делит
            // на длину = r, отдельно на радиус делить не нужно.
            hit.normal = hit.point.sub(s.center).normalize();
            hit.material = s.material;
        }
    }

    // ---- Треугольники ----
    for (const Triangle& tri : g_scene.triangles) {
        double t;
        Vec3   n;
        if (tri.rayIntersect(orig, dir, t, n) && t < bestDist) {
            bestDist = t;
            hit.point = orig.add(dir.mul(t));
            hit.normal = n;             // уже нормализована и развёрнута
            hit.material = tri.material;
        }
    }

    // ---- Шахматная плоскость y = -4 ----
    // Решаем orig.y + t·dir.y = -4  ⇒  t = -(orig.y + 4) / dir.y.
    // Если dir.y ≈ 0 — луч параллелен плоскости.
    if (std::fabs(dir.y) > 1e-3) {
        double d = -(orig.y + 4.0) / dir.y;
        Vec3   pt = orig.add(dir.mul(d));

        // Ограничиваем плоскость прямоугольником [-10,10]×[-30,-10]
        // по X/Z, чтобы шахматное поле не занимало весь кадр.
        if (d > 0 && std::fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30
            && d < bestDist) {
            bestDist = d;
            hit.point = pt;
            hit.normal = Vec3(0, 1, 0);

            // Шахматная доска: floor координат с шагом 2 юнита,
            // складываем x и z, смотрим младший бит.
            // +1000 — чтобы не уйти в отрицательные значения
            // при приведении к int.
            Vec3 col = ((((int)(0.5 * pt.x + 1000) + (int)(0.5 * pt.z)) & 1) != 0)
                ? Vec3(1.0, 1.0, 1.0)      // белая клетка
                : Vec3(1.0, 0.7, 0.3);     // оранжевая клетка

            // Плоскости назначаем "пустой" материал и подменяем ему
            // только диффузный цвет (затемнённый множителем 0.3).
            hit.material = Material();
            hit.material.diffuseColor = col.mul(0.3);
        }
    }

    // Условная дальняя плоскость отсечения — всё за 1000 считаем фоном.
    return bestDist < 1000.0;
}

// ----------------------------------------------------------------------------
// ГЛАВНАЯ РЕКУРСИВНАЯ ФУНКЦИЯ ТРАССИРОВКИ.
//
// Если луч ни во что не попал (или упёрлись в MAX_DEPTH) — возвращаем
// цвет неба. Иначе в точке попадания:
//   1. Считаем отражённый и преломлённый лучи.
//   2. Рекурсивно трассируем ОБА.
//   3. Для каждого источника света пускаем ТЕНЕВОЙ луч.
//   4. Суммируем диффузный (Ламберт) и зеркальный (Фонг) вклады.
//   5. Складываем всё, взвешивая через компоненты albedo.
// ----------------------------------------------------------------------------
static Vec3 castRay(const Vec3& orig, const Vec3& dir, int depth) {
    Hit hit;
    if (depth > MAX_DEPTH || !sceneIntersect(orig, dir, hit)) {
        return Vec3(0.2, 0.7, 0.8);   // цвет неба
    }

    const Vec3& point = hit.point;
    const Vec3& N = hit.normal;    // нормаль поверхности
    const Material& mat = hit.material;

    // ---------------- Отражённый и преломлённый лучи ----------------
    Vec3 reflectDir = reflect(dir, N).normalize();
    Vec3 refractDir = refract(dir, N, mat.refractiveIndex).normalize();

    // Смещаем начало вторичного луча на epsilon вдоль нормали,
    // чтобы он не "родился" ровно на поверхности и не поймал её
    // саму (иначе — артефакт "surface acne", поверхность затеняет
    // сама себя из-за ошибок округления).
    Vec3 reflectOrig = reflectDir.dot(N) < 0 ? point.sub(N.mul(1e-3)) : point.add(N.mul(1e-3));
    Vec3 refractOrig = refractDir.dot(N) < 0 ? point.sub(N.mul(1e-3)) : point.add(N.mul(1e-3));

    // Рекурсивные вызовы. Глубина растёт — рано или поздно
    // упрёмся в MAX_DEPTH и ветка оборвётся.
    Vec3 reflectColor = castRay(reflectOrig, reflectDir, depth + 1);
    Vec3 refractColor = castRay(refractOrig, refractDir, depth + 1);

    // ---------------- Прямое освещение и тени ----------------
    double diffInt = 0.0;   // суммарная диффузная интенсивность
    double specInt = 0.0;   // суммарный зеркальный блик

    for (const Light& light : g_scene.lights) {
        Vec3   lightDir = light.position.sub(point).normalize();
        double lightDist = light.position.sub(point).norm();

        // Теневой луч тоже смещаем от поверхности — см. про acne.
        Vec3 shadowOrig = lightDir.dot(N) < 0 ? point.sub(N.mul(1e-3))
            : point.add(N.mul(1e-3));

        Hit sh;
        if (sceneIntersect(shadowOrig, lightDir, sh) &&
            sh.point.sub(shadowOrig).norm() < lightDist) {
            // Кто-то стоит между нами и источником — точка в тени.
            continue;
        }

        // ЗАКОН ЛАМБЕРТА: яркость ∝ cos угла между нормалью и
        // направлением на свет, т.е. dot(N, L). max(0, ·)
        // отсекает свет, падающий "сзади".
        diffInt += light.intensity * std::max(0.0, lightDir.dot(N));

        // МОДЕЛЬ ФОНГА: блик тем ярче, чем ближе направление
        // "от поверхности" к отражению направления на свет.
        // Возведение в степень specularExponent делает блик узким.
        specInt += std::pow(std::max(0.0, -reflect(lightDir.neg(), N).dot(dir)),
            mat.specularExponent) * light.intensity;
    }

    // ---------------- Итоговая формула цвета ----------------
    //   C = albedo[0] · diffuseColor · diffInt      (рассеяние)
    //     + albedo[1] · (1,1,1)     · specInt       (блик)
    //     + albedo[2] · reflectColor                (отражение)
    //     + albedo[3] · refractColor                (преломление)
    return mat.diffuseColor.mul(diffInt * mat.albedo[0])
        .add(Vec3(1, 1, 1).mul(specInt * mat.albedo[1]))
        .add(reflectColor.mul(mat.albedo[2]))
        .add(refractColor.mul(mat.albedo[3]));
}

// Ограничение значения в [0, 1] — для финальной упаковки в 8-битный цвет.
static inline double clamp01(double v) {
    return std::max(0.0, std::min(1.0, v));
}

// ============================== ПУЛ ПОТОКОВ =================================
// Маленький синхронный пул потоков.
//
// Зачем свой, а не OpenMP/std::async?
//   * OpenMP потребовал бы флага /openmp, а мы хотим "чистый C++17".
//   * std::async каждый кадр создавал бы потоки заново — дорого.
//   * Наш пул создаёт N потоков ОДИН РАЗ и переиспользует их.
//
// Схема:
//   * main вызывает run(task).
//   * task кладётся в общий слот, счётчик done сбрасывается,
//     "поколение" gen увеличивается — воркеры просыпаются.
//   * каждый воркер получает свой idx (0..N-1) и общее N; он
//     выполняет task(idx, N). Разбиение работы — забота task.
//   * когда все отчитались, main просыпается через cvDone_.
//
// ВАЖНО: пул синхронный — нельзя вызвать run() из воркера.
// Нам это и не нужно, run() зовёт только главный поток.
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
            ++gen_;     // поднимаем воркеров, чтобы они увидели stop_
        }
        cv_.notify_all();
        for (auto& t : threads_) t.join();
    }

    void run(Task task) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            task_ = std::move(task);
            done_ = 0;
            ++gen_;     // поколение изменилось — воркеры проснутся
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
                // Спим, пока gen_ не изменится.
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

    int                       n_;       // число потоков
    int                       gen_;     // счётчик "поколений"
    int                       done_;    // сколько воркеров закончили
    bool                      stop_;    // сигнал завершения
    Task                      task_;    // текущая задача
    std::mutex                mtx_;
    std::condition_variable   cv_;      // воркеры ждут здесь
    std::condition_variable   cvDone_;  // main ждёт здесь
    std::vector<std::thread>  threads_;
};

// ====================== АНИМИРОВАННЫЕ МЕШИ ==================================
// "Меш" — набор треугольников. Чтобы анимировать фигуру (двигать +
// вращать), не пересобирая её каждый кадр, храним:
//   * ШАБЛОН МЕША — вершины относительно локального нуля (0,0,0);
//   * базовую позицию центра в мире;
//   * параметры анимации (фазу и скорость вращения);
//   * startIdx/triCount — какие треугольники в g_scene.triangles
//     этому мешу соответствуют.
//
// Каждый кадр в updateMeshes() мы пересчитываем мировые вершины как
//     world = center(t) + rotateY(local, angle(t)).
// Никаких матриц 4x4: вращение только вокруг Y, так что хватает
// одной тригонометрической формулы.
// ============================================================================
struct AnimatedMesh {
    int                   startIdx;    // индекс первого треугольника в scene.triangles
    int                   triCount;    // сколько треугольников
    Vec3                  baseCenter;  // базовая позиция центра
    double                phase;       // фаза для синусоид
    double                rotSpeed;    // рад/сек вокруг Y
    std::vector<Triangle> templateTris; // локальные вершины
};
static std::vector<AnimatedMesh> g_meshes;

// ----------------------------------------------------------------------------
// Поворот точки вокруг оси Y на угол a:
//     x' =  x·cos a + z·sin a
//     y' =  y
//     z' = -x·sin a + z·cos a
//
// Матрица поворота вокруг Y:
//     |  cos a  0  sin a |
//     |    0    1    0   |
//     | -sin a  0  cos a |
//
// Можно было бы собирать матрицу 3x3 и умножать — но для одной
// оси хватает 4 умножений и 2 сложений на точку.
// ----------------------------------------------------------------------------
static inline Vec3 rotateY(const Vec3& p, double a) {
    double c = std::cos(a), s = std::sin(a);
    return Vec3(c * p.x + s * p.z, p.y, -s * p.x + c * p.z);
}

// ----------------------------------------------------------------------------
// ФАБРИКИ ФИГУР. Возвращают вектор треугольников с вершинами
// относительно локального центра (0,0,0). Материал один на всю фигуру.
// ----------------------------------------------------------------------------

// Куб со стороной side с центром в локальном нуле.
// 8 угловых вершин, 6 граней, 12 треугольников (каждая грань — два
// треугольника по общей диагонали).
static std::vector<Triangle> makeCube(double side, const Material& mat) {
    std::vector<Triangle> tris;
    tris.reserve(12);
    double h = side * 0.5;

    // Углы куба. 0..3 — задняя грань (z = -h), 4..7 — передняя (z = +h).
    Vec3 v[8] = {
        Vec3(-h, -h, -h), Vec3(h, -h, -h), Vec3(h,  h, -h), Vec3(-h,  h, -h),
        Vec3(-h, -h,  h), Vec3(h, -h,  h), Vec3(h,  h,  h), Vec3(-h,  h,  h),
    };

    // Каждая грань — два треугольника по индексам в v[]. Порядок
    // вершин задаёт начальное направление нормали, но наш Triangle
    // всё равно потом разворачивает её навстречу лучу.
    static const int f[12][3] = {
        {1,2,6},{1,6,5},   // +X
        {0,4,7},{0,7,3},   // -X
        {3,7,6},{3,6,2},   // +Y
        {0,1,5},{0,5,4},   // -Y
        {4,5,6},{4,6,7},   // +Z
        {0,3,2},{0,2,1},   // -Z
    };
    for (int i = 0; i < 12; ++i)
        tris.push_back({ v[f[i][0]], v[f[i][1]], v[f[i][2]], mat });
    return tris;
}

// Пирамида: квадратное основание baseSide на плоскости y = 0,
// вершина на высоте height над центром основания.
// 4 боковые грани + 2 треугольника основания = 6 треугольников.
static std::vector<Triangle> makePyramid(double baseSide, double height,
    const Material& mat) {
    std::vector<Triangle> tris;
    tris.reserve(6);
    double h = baseSide * 0.5;

    Vec3 b0(-h, 0.0, -h);
    Vec3 b1(h, 0.0, -h);
    Vec3 b2(h, 0.0, h);
    Vec3 b3(-h, 0.0, h);
    Vec3 apex(0.0, height, 0.0);

    // Основание (квадрат из двух треугольников).
    tris.push_back({ b0, b1, b2, mat });
    tris.push_back({ b0, b2, b3, mat });
    // Четыре боковые грани.
    tris.push_back({ b0, b1, apex, mat });
    tris.push_back({ b1, b2, apex, mat });
    tris.push_back({ b2, b3, apex, mat });
    tris.push_back({ b3, b0, apex, mat });
    return tris;
}

// Регистрация меша в сцене. Копирует шаблон в g_scene.triangles
// (с начальным сдвигом в baseCenter) и запоминает соответствие
// для последующей анимации.
static void addAnimatedMesh(std::vector<Triangle> templateTris,
    const Vec3& center,
    double phase,
    double rotSpeed) {
    AnimatedMesh m;
    m.startIdx = (int)g_scene.triangles.size();
    m.triCount = (int)templateTris.size();
    m.baseCenter = center;
    m.phase = phase;
    m.rotSpeed = rotSpeed;
    m.templateTris = std::move(templateTris);

    // Кладём копии в сцену (стартовые позиции — без анимации).
    for (const Triangle& t : m.templateTris) {
        Triangle wt;
        wt.v0 = center.add(t.v0);
        wt.v1 = center.add(t.v1);
        wt.v2 = center.add(t.v2);
        wt.material = t.material;
        g_scene.triangles.push_back(wt);
    }
    g_meshes.push_back(std::move(m));
}

// Пересчёт мировых вершин всех мешей по времени t.
static void updateMeshes(double t) {
    for (const AnimatedMesh& m : g_meshes) {
        double fi = m.phase;
        // Смещение центра — те же синусоиды, что и у сфер,
        // но со своими фазами (фаза = m.phase).
        double dx = std::sin(t * 0.8 + fi * 1.7) * 1.6;
        double dy = std::sin(t * 1.1 + fi * 2.1) * 1.1;
        double dz = std::cos(t * 0.6 + fi * 1.3) * 1.4;
        Vec3 c = m.baseCenter.add(Vec3(dx, dy, dz));

        double angle = t * m.rotSpeed + fi;

        for (int k = 0; k < m.triCount; ++k) {
            const Triangle& src = m.templateTris[k];
            Triangle& dst = g_scene.triangles[m.startIdx + k];
            // Мировая вершина = центр + rotateY(локальная вершина).
            dst.v0 = c.add(rotateY(src.v0, angle));
            dst.v1 = c.add(rotateY(src.v1, angle));
            dst.v2 = c.add(rotateY(src.v2, angle));
            dst.material = src.material;
        }
    }
}

// ============================ ГЛОБАЛЬНОЕ СОСТОЯНИЕ ==========================
// WinAPI WndProc — это C-указатель, лямбду с захватом туда не положишь.
// Поэтому состояние выносим в файл-скоуп.
static HWND        g_hwnd = nullptr;   // окно
static HDC         g_memDC = nullptr;   // off-screen DC
static HBITMAP     g_hBitmap = nullptr;   // DIB-секция (кадровый буфер)
static uint32_t* g_dibBits = nullptr;   // указатель на пиксели DIB
static HFONT       g_font = nullptr;   // шрифт для текста FPS
static bool        g_dragging = false;    // тащат ли сферу мышью сейчас
static ThreadPool* g_pool = nullptr;   // пул потоков

// ============================================================================
// РЕНДЕР ОДНОГО КАДРА В БУФЕР.
// Разбиение работы: воркер idx обрабатывает строки j = idx, idx+N, idx+2N...
// Строки примерно одинаковой стоимости — нет "горячих" полос.
// ============================================================================
static void renderToBuffer(uint32_t* pixels) {
    // Предвычислим всё, что не зависит от пикселя.
    const double tanHalfFov = std::tan(FOV / 2.0);   // tg(FOV/2)
    const double aspect = (double)WIDTH / (double)HEIGHT;

    g_pool->run([pixels, tanHalfFov, aspect](int idx, int nThreads) {
        for (int j = idx; j < HEIGHT; j += nThreads) {
            // Экранные координаты → координаты в мире.
            // j = 0 — верх экрана, j = HEIGHT — низ.
            // Формула даёт y ∈ [-tg(FOV/2), +tg(FOV/2)] со знаком
            // "верх = +Y" (инвертируем, чтобы картинка была
            // правильной ориентации).
            // +0.5 — сдвиг к центру ПИКСЕЛЯ (а не его углу).
            double y = -(2.0 * (j + 0.5) / HEIGHT - 1.0) * tanHalfFov;
            uint32_t* row = pixels + (size_t)j * WIDTH;

            for (int i = 0; i < WIDTH; ++i) {
                // По X добавляем aspect, чтобы круглые объекты не
                // превращались в эллипсы.
                double x = (2.0 * (i + 0.5) / WIDTH - 1.0) * tanHalfFov * aspect;

                // Направление луча из камеры в (0,0,0), смотрящей в -Z.
                Vec3 dir = Vec3(x, y, -1.0).normalize();
                Vec3 c = castRay(Vec3(0, 0, 0), dir, 0);

                // Простейший тонмаппинг: если канал > 1, делим весь
                // цвет на максимум — сохраняя оттенок. Спасает от
                // "пересвета в белый" на ярких бликах.
                double mx = std::max(c.x, std::max(c.y, c.z));
                if (mx > 1.0) c = c.mul(1.0 / mx);

                // Упаковка в 8-битный цвет.
                int r = (int)(255.0 * clamp01(c.x));
                int g = (int)(255.0 * clamp01(c.y));
                int b = (int)(255.0 * clamp01(c.z));

                // DIB 32bpp хранит BGRA, но в uint32 это 0xAARRGGBB
                // (little-endian): alpha в старшем байте, потом R, G, B.
                row[i] = 0xFF000000u | ((uint32_t)r << 16) |
                    ((uint32_t)g << 8) | (uint32_t)b;
            }
        }
        });
}

// ============================================================================
// ПРЕОБРАЗОВАНИЕ КООРДИНАТ МЫШИ → МИРОВЫЕ.
//
// Мышь даёт (mx, my) в пикселях клиентской области. Хотим понять,
// где в 3D-пространстве "лежит" курсор, ЕСЛИ бы он был на
// плоскости z = MOUSE_SPHERE_Z.
//
// Формулы — те же, что при генерации лучей, но без normalize:
// получаем точку на плоскости z = -1, умножаем на глубину —
// получаем точку с z = MOUSE_SPHERE_Z.
// ============================================================================
static Vec3 screenToWorld(double mx, double my) {
    double nx = mx / (double)(WIDTH * SCALE);   // 0..1 по ширине окна
    double ny = my / (double)(HEIGHT * SCALE);  // 0..1 по высоте

    double tx = (2.0 * nx - 1.0) * std::tan(FOV / 2.0) * WIDTH / (double)HEIGHT;
    double ty = -(2.0 * ny - 1.0) * std::tan(FOV / 2.0);

    double depth = -MOUSE_SPHERE_Z;   // положительное расстояние от камеры
    return Vec3(tx * depth, ty * depth, MOUSE_SPHERE_Z);
}

// ============================== WNDPROC =====================================
// Оконная процедура. Windows присылает сюда все сообщения окна.
// Рендер идёт в главном цикле 60 раз в секунду; WM_PAINT только
// блитит уже готовый кадр (нужен при появлении окна и т.п.).
// ============================================================================
static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_LBUTTONDOWN: {
        g_dragging = true;
        // LPARAM упаковывает (x, y) в младшие/старшие 16 бит.
        // Макросы корректно извлекают их с учётом знака.
        int mx = GET_X_LPARAM(lp);
        int my = GET_Y_LPARAM(lp);
        g_scene.spheres[MOUSE_SPHERE_IDX].center = screenToWorld(mx, my);
        return 0;
    }
    case WM_MOUSEMOVE: {
        if (g_dragging) {
            int mx = GET_X_LPARAM(lp);
            int my = GET_Y_LPARAM(lp);
            g_scene.spheres[MOUSE_SPHERE_IDX].center = screenToWorld(mx, my);
        }
        return 0;
    }
    case WM_LBUTTONUP:
        g_dragging = false;
        return 0;

    case WM_ERASEBKGND:
        // Возвращаем 1: "я сам всё стёр" — Windows не будет
        // заливать фон, не будет мерцания.
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
        // Кинуть WM_QUIT, чтобы главный цикл завершился.
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wp, lp);
}

// ============================== WINMAIN =====================================
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    // HiDPI: Windows по умолчанию растягивает окна на мониторах
    // с масштабом ≠ 100%. SetProcessDPIAware говорит: "не трогай
    // мои пиксели, я сам". Окно будет ровно 480×320 физических
    // пикселей, картинка — резкой (без интерполяции).
    SetProcessDPIAware();

    // --------------------------- МАТЕРИАЛЫ ---------------------------------
    // (refractiveIndex, albedo[0..3], diffuseColor, specularExponent)
    //
    //   ivory        — слоновая кость: слегка рассеивающий, слабый блик
    //   glass        — стекло: сильный refract (0.8), средний reflect (0.1)
    //   redRubber    — красная резина: чисто диффузная, глухой блик
    //   mirror       — зеркало: почти чистый reflect (0.8),
    //                  specular усилен до 10 — эффект яркого солнца
    //   gold         — тёплый металл: диффуз + сильный блик + отражение
    //   cobalt       — синий металлик: чуть более матовый, чем золото
    Material ivory(1.0, 0.6, 0.3, 0.1, 0.0, Vec3(0.4, 0.4, 0.3), 50.0);
    Material glass(1.5, 0.0, 0.5, 0.1, 0.8, Vec3(0.6, 0.7, 0.8), 125.0);
    Material redRubber(1.0, 0.9, 0.1, 0.0, 0.0, Vec3(0.3, 0.1, 0.1), 10.0);
    Material mirror(1.0, 0.0, 10.0, 0.8, 0.0, Vec3(1.0, 1.0, 1.0), 1425.0);
    Material gold(1.0, 0.55, 0.55, 0.35, 0.0, Vec3(0.95, 0.65, 0.20), 300.0);
    Material cobalt(1.0, 0.65, 0.35, 0.20, 0.0, Vec3(0.20, 0.30, 0.85), 180.0);

    // --------------------------- СФЕРЫ -------------------------------------
    // Первая — та, которой управляет мышь (индекс 0).
    // Остальные анимируются по синусоидам вокруг base-позиций.
    g_scene.spheres.emplace_back(Vec3(7.0, 5.0, -18.0), 4.0, mirror);
    g_scene.spheres.emplace_back(Vec3(-1.0, -1.5, -12.0), 2.0, glass);
    g_scene.spheres.emplace_back(Vec3(-3.0, 0.0, -16.0), 2.0, ivory);
    g_scene.spheres.emplace_back(Vec3(1.5, -0.5, -18.0), 3.0, redRubber);

    // Три источника разного положения и интенсивности —
    // чтобы тени и блики были "многогранными".
    g_scene.lights.push_back({ Vec3(-20.0, 20.0,  20.0), 1.5 });
    g_scene.lights.push_back({ Vec3(30.0, 50.0, -25.0), 1.8 });
    g_scene.lights.push_back({ Vec3(30.0, 20.0,  30.0), 1.7 });

    // Запомнили стартовые позиции — от них считаем смещения
    // при анимации, чтобы сферы не "уползали" от кадра к кадру.
    g_scene.basePositions.resize(g_scene.spheres.size());
    for (size_t i = 0; i < g_scene.spheres.size(); ++i)
        g_scene.basePositions[i] = g_scene.spheres[i].center;

    // --------------------------- ФИГУРЫ ------------------------------------
    // Куб и пирамида — по обе стороны от сфер, на глубине ~-14,
    // чтобы не пересекаться со сферами. phase/rotSpeed подобраны так,
    // чтобы фигуры двигались вразнобой и вращались с разной скоростью.
    addAnimatedMesh(makeCube(4.0, gold), Vec3(3.5, 1.5, -14.0),
        /*phase=*/0.0,  /*rotSpeed=*/ 0.7);
    addAnimatedMesh(makePyramid(4.0, 5.0, cobalt), Vec3(-5.5, 0.0, -13.5),
        /*phase=*/1.7,  /*rotSpeed=*/-0.5);

    // --------------------------- ПУЛ ПОТОКОВ -------------------------------
    // hardware_concurrency() = число логических ядер. Для трассировки
    // это нормально: у нас нет разделяемых ресурсов (воркеры пишут
    // в разные строки, значит и в разные кэш-линии).
    unsigned hw = std::thread::hardware_concurrency();
    int nThreads = (hw == 0) ? 1 : (int)hw;
    ThreadPool pool(nThreads);
    g_pool = &pool;

    // --------------------------- СОЗДАНИЕ ОКНА -----------------------------
    // Убираем WS_THICKFRAME (изменение размера) и WS_MAXIMIZEBOX —
    // фиксированный размер, содержимое всегда совпадает с DIB.
    const DWORD style = (WS_OVERLAPPEDWINDOW & ~(WS_THICKFRAME | WS_MAXIMIZEBOX));

    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(wc);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    wc.hbrBackground = nullptr;   // фон не нужен (см. WM_ERASEBKGND)
    wc.lpszClassName = L"TinyRayTracerFXClass";
    RegisterClassExW(&wc);

    // AdjustWindowRect добавляет к клиентской области рамку и
    // заголовок — так клиентская часть будет ровно WIDTH×HEIGHT.
    RECT rc = { 0, 0, WIDTH * SCALE, HEIGHT * SCALE };
    AdjustWindowRect(&rc, style, FALSE);

    g_hwnd = CreateWindowExW(
        0, L"TinyRayTracerFXClass",
        L"Tiny Ray Tracer — C++/Win32 (sphere/cube/pyramid)",
        style,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right - rc.left, rc.bottom - rc.top,
        nullptr, nullptr, hInstance, nullptr);

    if (!g_hwnd) return 1;

    // --------------------------- DIB-СЕКЦИЯ --------------------------------
    // DIB = Device-Independent Bitmap. CreateDIBSection даёт нам
    // и HBITMAP (для BitBlt), и прямой указатель на пиксели —
    // пишем в буфер, одним BitBlt'ом выводим на экран. Быстрее,
    // чем поштучно SetPixel.
    HDC hdcScreen = GetDC(g_hwnd);

    BITMAPINFO bmi = {};
    bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmi.bmiHeader.biWidth = WIDTH;
    // ОТРИЦАТЕЛЬНАЯ высота = top-down DIB: строка 0 — верхняя.
    // Удобно: экранные координаты совпадают с индексами массива.
    bmi.bmiHeader.biHeight = -HEIGHT;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;   // BGRA по 8 бит
    bmi.bmiHeader.biCompression = BI_RGB;

    void* bits = nullptr;
    g_hBitmap = CreateDIBSection(hdcScreen, &bmi, DIB_RGB_COLORS, &bits, nullptr, 0);
    g_dibBits = reinterpret_cast<uint32_t*>(bits);

    // Память-DC + выбранная в него DIB-секция. Всё рисование
    // (в т.ч. текст FPS) идёт в наш буфер в RAM.
    g_memDC = CreateCompatibleDC(hdcScreen);
    SelectObject(g_memDC, g_hBitmap);
    ReleaseDC(g_hwnd, hdcScreen);

    // --------------------------- ШРИФТ ДЛЯ FPS -----------------------------
    // Consolas моноширинный — цифры не "дёргаются" при смене FPS.
    g_font = CreateFontW(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, FF_MODERN, L"Consolas");

    ShowWindow(g_hwnd, nCmdShow);
    UpdateWindow(g_hwnd);

    // ========================================================================
    // ГЛАВНЫЙ ЦИКЛ
    //  1. Неблокирующая обработка всех сообщений (PeekMessage).
    //  2. Подсчёт FPS (скользящее окно на 30 кадров).
    //  3. Анимация сфер (синусоиды).
    //  4. Анимация мешей (смещение + вращение вокруг Y).
    //  5. Параллельный рендер кадра в DIB.
    //  6. Текст FPS.
    //  7. Блит на экран.
    // ========================================================================
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    static constexpr int FPS_WINDOW = 30;
    long long frameTimes[FPS_WINDOW] = {};
    int    frameIndex = 0;   // куда писать следующий отсчёт
    int    framesFilled = 0;   // сколько набрано (до FPS_WINDOW)
    double fps = 0.0;

    bool running = true;
    while (running) {
        // ---- обработка сообщений Windows ----
        MSG msg;
        while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) { running = false; break; }
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if (!running) break;

        // ---- измерение FPS ----
        auto now = clock::now();
        long long nowNs =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();

        frameTimes[frameIndex] = nowNs;
        frameIndex = (frameIndex + 1) % FPS_WINDOW;
        if (framesFilled < FPS_WINDOW) framesFilled++;

        if (framesFilled >= 2) {
            // Самый старый из "живых" отсчётов.
            int oldestIdx = (frameIndex - framesFilled + FPS_WINDOW) % FPS_WINDOW;
            long long dt = nowNs - frameTimes[oldestIdx];   // нс
            if (dt > 0)
                fps = (framesFilled - 1) * 1.0e9 / (double)dt;
        }

        double t = std::chrono::duration<double>(now - t0).count();

        // ---- анимация сфер ----
        // Каждая сфера колеблется по синусоидам со своей фазой
        // и амплитудой — независимые красивые траектории.
        for (size_t i = 0; i < g_scene.spheres.size(); ++i) {
            if ((int)i == MOUSE_SPHERE_IDX) continue;   // эту не трогаем
            const Vec3 base = g_scene.basePositions[i];
            double fi = (double)i;
            double dx = std::sin(t * 0.8 + fi * 1.7) * (1.7 + fi * 0.25);
            double dy = std::sin(t * 1.1 + fi * 2.1) * 1.4;
            double dz = std::cos(t * 0.6 + fi * 1.3) * (1.4 + fi * 0.20);
            g_scene.spheres[i].center = Vec3(base.x + dx, base.y + dy, base.z + dz);
        }

        // ---- анимация мешей (куб, пирамида) ----
        updateMeshes(t);

        // ---- рендер кадра ----
        renderToBuffer(g_dibBits);

        // ---- текст FPS ----
        // Рисуем прямо в DIB через GDI: SetBkMode(TRANSPARENT) —
        // чтобы не затирать фон прямоугольником вокруг букв.
        char fpsText[64];
        std::snprintf(fpsText, sizeof(fpsText), "FPS: %.1f", fps);

        SetBkMode(g_memDC, TRANSPARENT);
        SetTextColor(g_memDC, RGB(255, 255, 255));
        HFONT oldFont = (HFONT)SelectObject(g_memDC, g_font);

        SIZE sz = {};
        GetTextExtentPoint32A(g_memDC, fpsText, (int)std::strlen(fpsText), &sz);

        const int padX = 8, padY = 6;
        int boxW = sz.cx + 2 * padX;
        int boxX = WIDTH * SCALE - boxW - 10;   // правый верхний угол
        int boxY = 10;

        TextOutA(g_memDC, boxX + padX, boxY + padY,
            fpsText, (int)std::strlen(fpsText));
        SelectObject(g_memDC, oldFont);

        // ---- вывод кадра на экран ----
        // SRCCOPY — просто копирование пиксель-в-пиксель.
        HDC hdc = GetDC(g_hwnd);
        BitBlt(hdc, 0, 0, WIDTH * SCALE, HEIGHT * SCALE,
            g_memDC, 0, 0, SRCCOPY);
        ReleaseDC(g_hwnd, hdc);
    }

    // --------------------------- ОЧИСТКА РЕСУРСОВ --------------------------
    // Важен порядок: пул потоков должен быть уничтожен ДО глобальных
    // буферов, иначе воркеры могут обратиться к освобождённой памяти.
    g_pool = nullptr;

    if (g_font)    DeleteObject(g_font);
    if (g_memDC)   DeleteDC(g_memDC);
    if (g_hBitmap) DeleteObject(g_hBitmap);

    return 0;
}