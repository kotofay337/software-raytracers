package org.example.graphics;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.PixelFormat;
import javafx.scene.image.WritableImage;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.stage.Stage;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class TinyRayTracerFX extends Application {

   static final int WIDTH      = 640;
   static final int HEIGHT     = 400;
   static final int SCALE      = 1;
   static final int MAX_DEPTH  = 6;
   static final double FOV     = Math.PI / 2;

   static final double MOUSE_SPHERE_Z     = -16.0;
   static final int    MOUSE_SPHERE_INDEX = 0;

   private static final int FPS_WINDOW = 30;
   private final long[] frameTimes = new long[FPS_WINDOW];
   private int    frameIndex = 0;
   private int    framesFilled = 0;
   private double fps = 0;

   // Общие константы/кэш для рендера
   private static final Vec3 ORIGIN = new Vec3(0, 0, 0);
   private static final double TAN_HALF_FOV = Math.tan(FOV / 2);
   private static final double ASPECT = WIDTH / (double) HEIGHT;

   // Материал пола — переиспользуем, чтобы не аллоцировать в горячем цикле
   private static final Material FLOOR_WHITE  =
           new Material(1.0, new double[]{1, 0, 0, 0}, new Vec3(0.30, 0.30, 0.30), 0);
   private static final Material FLOOR_ORANGE =
           new Material(1.0, new double[]{1, 0, 0, 0}, new Vec3(0.30, 0.21, 0.09), 0);

   // ==================== Vec3 ====================
   static final class Vec3 {
      final double x, y, z;
      Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
      Vec3() { this(0, 0, 0); }
      Vec3 add(Vec3 o) { return new Vec3(x + o.x, y + o.y, z + o.z); }
      Vec3 sub(Vec3 o) { return new Vec3(x - o.x, y - o.y, z - o.z); }
      Vec3 mul(double s) { return new Vec3(x * s, y * s, z * s); }
      Vec3 neg() { return new Vec3(-x, -y, -z); }
      double dot(Vec3 o) { return x * o.x + y * o.y + z * o.z; }
      double norm() { return Math.sqrt(x * x + y * y + z * z); }
      Vec3 normalize() {
         double n = Math.sqrt(x * x + y * y + z * z);
         return n == 0 ? new Vec3() : new Vec3(x / n, y / n, z / n);
      }
   }

   static Vec3 reflect(Vec3 i, Vec3 n) {
      double d = 2.0 * (i.x * n.x + i.y * n.y + i.z * n.z);
      return new Vec3(i.x - n.x * d, i.y - n.y * d, i.z - n.z * d);
   }

   static Vec3 refract(Vec3 i, Vec3 n, double eta) {
      double cosi = -Math.max(-1.0, Math.min(1.0, i.dot(n)));
      double etai = 1.0, etat = eta;
      Vec3 nn = n;
      if (cosi < 0) { cosi = -cosi; double t = etai; etai = etat; etat = t; nn = n.neg(); }
      double r = etai / etat;
      double k = 1.0 - r * r * (1.0 - cosi * cosi);
      if (k < 0) return new Vec3();
      double s = r * cosi - Math.sqrt(k);
      return new Vec3(i.x * r + nn.x * s, i.y * r + nn.y * s, i.z * r + nn.z * s);
   }

   // ==================== Material ====================
   static final class Material {
      final double refractiveIndex;
      final double[] albedo;
      final Vec3 diffuseColor;
      final double specularExponent;

      Material(double ri, double[] albedo, Vec3 color, double spec) {
         this.refractiveIndex  = ri;
         this.albedo           = albedo.clone();
         this.diffuseColor     = color;
         this.specularExponent = spec;
      }
      Material() { this(1.0, new double[]{1, 0, 0, 0}, new Vec3(), 0.0); }
   }

   // ==================== Sphere ====================
   static final class Sphere {
      Vec3 center;
      final double radius;
      final double radius2;
      final Material material;

      Sphere(Vec3 c, double r, Material m) {
         center = c; radius = r; radius2 = r * r; material = m;
      }

      /** @return t > 0 при попадании, иначе -1. Без аллокаций. */
      double rayIntersect(Vec3 orig, Vec3 dir) {
         double lx = center.x - orig.x;
         double ly = center.y - orig.y;
         double lz = center.z - orig.z;

         double tca = lx * dir.x + ly * dir.y + lz * dir.z;
         double d2  = lx * lx + ly * ly + lz * lz - tca * tca;
         if (d2 > radius2) return -1;

         double thc = Math.sqrt(radius2 - d2);
         double t0  = tca - thc;
         double t1  = tca + thc;
         if (t0 < 0) t0 = t1;
         if (t0 < 0) return -1;
         return t0;
      }

      /** Быстрая проверка «есть ли пересечение в интервале (0, maxT)». */
      boolean hitInRange(Vec3 orig, Vec3 dir, double maxT) {
         double lx = center.x - orig.x;
         double ly = center.y - orig.y;
         double lz = center.z - orig.z;

         double tca = lx * dir.x + ly * dir.y + lz * dir.z;
         double d2  = lx * lx + ly * ly + lz * lz - tca * tca;
         if (d2 > radius2) return false;

         double thc = Math.sqrt(radius2 - d2);
         double t0  = tca - thc;
         double t1  = tca + thc;
         if (t0 > 1e-3 && t0 < maxT) return true;
         if (t1 > 1e-3 && t1 < maxT) return true;
         return false;
      }
   }

   static final class Light {
      final Vec3 position;
      final double intensity;
      Light(Vec3 p, double i) { position = p; intensity = i; }
   }

   static final class Hit {
      Vec3     point;
      Vec3     normal;
      Material material;
   }

   // ==================== Сцена ====================
   final List<Sphere> spheres = new ArrayList<>();
   final List<Light>  lights  = new ArrayList<>();
   Vec3[] basePositions;
   Vec3[] rayDirs;    // предвычисленные направления лучей камеры

   // Проверка пересечения сцены и заполнение Hit. Возвращает true при попадании.
   boolean sceneIntersect(Vec3 orig, Vec3 dir, Hit hit) {
      double bestT = Double.MAX_VALUE;
      Sphere bestS = null;

      int n = spheres.size();
      for (int i = 0; i < n; i++) {
         Sphere s = spheres.get(i);
         double t = s.rayIntersect(orig, dir);
         if (t > 0 && t < bestT) {
            bestT = t;
            bestS = s;
         }
      }

      // Пол
      double floorT = Double.MAX_VALUE;
      double floorPx = 0, floorPz = 0;
      if (Math.abs(dir.y) > 1e-3) {
         double d = -(orig.y + 4.0) / dir.y;
         if (d > 0) {
            double px = orig.x + dir.x * d;
            double pz = orig.z + dir.z * d;
            if (Math.abs(px) < 10 && pz < -10 && pz > -30 && d < bestT) {
               floorT = d; floorPx = px; floorPz = pz;
            }
         }
      }

      if (floorT < bestT && floorT < 1000) {
         hit.point    = new Vec3(orig.x + dir.x * floorT, orig.y + dir.y * floorT, orig.z + dir.z * floorT);
         hit.normal   = new Vec3(0, 1, 0);
         int cell = ((int)(0.5 * floorPx + 1000) + (int)(0.5 * floorPz));
         hit.material = (cell & 1) != 0 ? FLOOR_WHITE : FLOOR_ORANGE;
         return true;
      }

      if (bestS != null && bestT < 1000) {
         double px = orig.x + dir.x * bestT;
         double py = orig.y + dir.y * bestT;
         double pz = orig.z + dir.z * bestT;
         hit.point  = new Vec3(px, py, pz);
         double nx = px - bestS.center.x;
         double ny = py - bestS.center.y;
         double nz = pz - bestS.center.z;
         double invN = 1.0 / Math.sqrt(nx * nx + ny * ny + nz * nz);
         hit.normal   = new Vec3(nx * invN, ny * invN, nz * invN);
         hit.material = bestS.material;
         return true;
      }

      return false;
   }

   /** Только проверка «есть ли препятствие между orig и maxDist вдоль dir». */
   boolean isShadowed(Vec3 orig, Vec3 dir, double maxDist) {
      int n = spheres.size();
      for (int i = 0; i < n; i++) {
         if (spheres.get(i).hitInRange(orig, dir, maxDist)) return true;
      }
      if (Math.abs(dir.y) > 1e-3) {
         double d = -(orig.y + 4.0) / dir.y;
         if (d > 1e-3 && d < maxDist) {
            double px = orig.x + dir.x * d;
            double pz = orig.z + dir.z * d;
            if (Math.abs(px) < 10 && pz < -10 && pz > -30) return true;
         }
      }
      return false;
   }

   Vec3 castRay(Vec3 orig, Vec3 dir, int depth) {
      if (depth > MAX_DEPTH) {
         return new Vec3(0.2, 0.7, 0.8);
      }

      Hit hit = new Hit();
      if (!sceneIntersect(orig, dir, hit)) {
         return new Vec3(0.2, 0.7, 0.8);
      }

      Vec3 point = hit.point;
      Vec3 N     = hit.normal;
      Material mat = hit.material;

      // Отражение — только если нужно
      Vec3 reflectColor = new Vec3();
      double wRefl = mat.albedo[2];
      if (wRefl > 0) {
         Vec3 reflectDir = reflect(dir, N).normalize();
         double eps = reflectDir.dot(N) < 0 ? -1e-3 : 1e-3;
         Vec3 reflectOrig = new Vec3(point.x + N.x * eps, point.y + N.y * eps, point.z + N.z * eps);
         reflectColor = castRay(reflectOrig, reflectDir, depth + 1);
      }

      // Преломление — только если нужно
      Vec3 refractColor = new Vec3();
      double wRefr = mat.albedo[3];
      if (wRefr > 0) {
         Vec3 refractDir = refract(dir, N, mat.refractiveIndex).normalize();
         double eps = refractDir.dot(N) < 0 ? -1e-3 : 1e-3;
         Vec3 refractOrig = new Vec3(point.x + N.x * eps, point.y + N.y * eps, point.z + N.z * eps);
         refractColor = castRay(refractOrig, refractDir, depth + 1);
      }

      double diffInt = 0, specInt = 0;
      int lightCount = lights.size();
      for (int li = 0; li < lightCount; li++) {
         Light light = lights.get(li);
         double lx = light.position.x - point.x;
         double ly = light.position.y - point.y;
         double lz = light.position.z - point.z;
         double lightDistSq = lx * lx + ly * ly + lz * lz;
         double lightDist = Math.sqrt(lightDistSq);
         double invDist = 1.0 / lightDist;
         double dirx = lx * invDist, diry = ly * invDist, dirz = lz * invDist;

         double ndl = dirx * N.x + diry * N.y + dirz * N.z;
         double eps = ndl < 0 ? -1e-3 : 1e-3;
         Vec3 shadowOrig = new Vec3(point.x + N.x * eps, point.y + N.y * eps, point.z + N.z * eps);

         if (isShadowed(shadowOrig, new Vec3(dirx, diry, dirz), lightDist)) continue;

         if (ndl > 0) diffInt += light.intensity * ndl;

         if (mat.specularExponent > 0) {
            // reflect(-lightDir, N) = -lightDir - 2*((-lightDir)·N)*N
            double ldn = -ndl;
            double rx = -dirx - 2.0 * ldn * N.x;
            double ry = -diry - 2.0 * ldn * N.y;
            double rz = -dirz - 2.0 * ldn * N.z;
            double rd = -(rx * dir.x + ry * dir.y + rz * dir.z);
            if (rd > 0) specInt += Math.pow(rd, mat.specularExponent) * light.intensity;
         }
      }

      double a0 = mat.albedo[0], a1 = mat.albedo[1];
      double dcx = mat.diffuseColor.x, dcy = mat.diffuseColor.y, dcz = mat.diffuseColor.z;

      double r = dcx * diffInt * a0 + specInt * a1 + reflectColor.x * wRefl + refractColor.x * wRefr;
      double g = dcy * diffInt * a0 + specInt * a1 + reflectColor.y * wRefl + refractColor.y * wRefr;
      double b = dcz * diffInt * a0 + specInt * a1 + reflectColor.z * wRefl + refractColor.z * wRefr;

      return new Vec3(r, g, b);
   }

   /** Предвычисление направлений лучей камеры. */
   void precomputeRayDirs() {
      rayDirs = new Vec3[WIDTH * HEIGHT];
      for (int j = 0; j < HEIGHT; j++) {
         double y = -(2 * (j + 0.5) / HEIGHT - 1) * TAN_HALF_FOV;
         int row = j * WIDTH;
         for (int i = 0; i < WIDTH; i++) {
            double x = (2 * (i + 0.5) / WIDTH - 1) * TAN_HALF_FOV * ASPECT;
            double inv = 1.0 / Math.sqrt(x * x + y * y + 1.0);
            rayDirs[row + i] = new Vec3(x * inv, y * inv, -inv);
         }
      }
   }

   void renderToBuffer(int[] pixels) {
      final Vec3[] dirs = this.rayDirs;
      IntStream.range(0, HEIGHT).parallel().forEach(j -> {
         int row = j * WIDTH;
         for (int i = 0; i < WIDTH; i++) {
            Vec3 dir = dirs[row + i];
            Vec3 c   = castRay(ORIGIN, dir, 0);

            double max = Math.max(c.x, Math.max(c.y, c.z));
            double scale = max > 1.0 ? 1.0 / max : 1.0;

            int r = (int) (255 * clamp01(c.x * scale));
            int g = (int) (255 * clamp01(c.y * scale));
            int b = (int) (255 * clamp01(c.z * scale));
            pixels[row + i] = 0xFF000000 | (r << 16) | (g << 8) | b;
         }
      });
   }

   static double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

   // ==================== JavaFX ====================
   @Override
   public void start(Stage stage) {
      Material ivory     = new Material(1.0, new double[]{0.6, 0.3, 0.1, 0.0}, new Vec3(0.4, 0.4, 0.3),   50);
      Material glass     = new Material(1.5, new double[]{0.0, 0.5, 0.1, 0.8}, new Vec3(0.6, 0.7, 0.8),  125);
      Material redRubber = new Material(1.0, new double[]{0.9, 0.1, 0.0, 0.0}, new Vec3(0.3, 0.1, 0.1),   10);
      Material mirror    = new Material(1.0, new double[]{0.0, 10.0, 0.8, 0.0}, new Vec3(1.0, 1.0, 1.0), 1425);

      spheres.add(new Sphere(new Vec3( 7.0,  5.0, -18), 4, mirror));
      spheres.add(new Sphere(new Vec3(-3.0,  0.0, -16), 2, ivory));
      spheres.add(new Sphere(new Vec3(-1.0, -1.5, -12), 2, glass));
      spheres.add(new Sphere(new Vec3( 1.5, -0.5, -18), 3, redRubber));

      lights.add(new Light(new Vec3(-20, 20,  20), 1.5));
      lights.add(new Light(new Vec3( 30, 50, -25), 1.8));
      lights.add(new Light(new Vec3( 30, 20,  30), 1.7));

      basePositions = new Vec3[spheres.size()];
      for (int i = 0; i < spheres.size(); i++) basePositions[i] = spheres.get(i).center;

      precomputeRayDirs();

      Canvas canvas = new Canvas(WIDTH * SCALE, HEIGHT * SCALE);
      GraphicsContext gc = canvas.getGraphicsContext2D();

      WritableImage image = new WritableImage(WIDTH, HEIGHT);
      int[] pixels = new int[WIDTH * HEIGHT];
      PixelFormat<java.nio.IntBuffer> fmt = PixelFormat.getIntArgbInstance();

      // ---------- Мышь ----------
      final boolean[] dragging = {false};

      java.util.function.BiFunction<Double, Double, Vec3> screenToWorld = (mx, my) -> {
         double nx =  mx / (double) (WIDTH  * SCALE);
         double ny =  my / (double) (HEIGHT * SCALE);
         double tx =  (2 * nx - 1) * TAN_HALF_FOV * ASPECT;
         double ty = -(2 * ny - 1) * TAN_HALF_FOV;
         double depth = -MOUSE_SPHERE_Z;
         return new Vec3(tx * depth, ty * depth, MOUSE_SPHERE_Z);
      };

      canvas.setOnMousePressed(e -> {
         dragging[0] = true;
         spheres.get(MOUSE_SPHERE_INDEX).center = screenToWorld.apply(e.getX(), e.getY());
      });
      canvas.setOnMouseDragged(e -> {
         if (!dragging[0]) return;
         spheres.get(MOUSE_SPHERE_INDEX).center = screenToWorld.apply(e.getX(), e.getY());
      });
      canvas.setOnMouseReleased(e -> dragging[0] = false);

      StackPane root = new StackPane(canvas);
      stage.setScene(new Scene(root));
      stage.setTitle("Tiny Ray Tracer — JavaFX (1 сфера управляется мышью)");
      stage.setResizable(false);
      stage.show();

      // ---------- Анимация ----------
      Font fpsFont = Font.font("Monospaced", 16);
      gc.setFont(fpsFont);

      AnimationTimer timer = new AnimationTimer() {
         long t0 = 0;
         @Override
         public void handle(long now) {
            // ---------- измерение FPS ----------
            frameTimes[frameIndex] = now;
            frameIndex = (frameIndex + 1) % FPS_WINDOW;
            if (framesFilled < FPS_WINDOW) framesFilled++;

            if (framesFilled >= 2) {
               int oldestIdx = (frameIndex - framesFilled + FPS_WINDOW) % FPS_WINDOW;
               long dt = now - frameTimes[oldestIdx];
               if (dt > 0) fps = (framesFilled - 1) * 1_000_000_000.0 / dt;
            }

            if (t0 == 0) t0 = now;
            double t = (now - t0) / 1e9;

            // ---------- анимация сфер ----------
            int n = spheres.size();
            for (int i = 0; i < n; i++) {
               if (i == MOUSE_SPHERE_INDEX) continue;
               Vec3 base = basePositions[i];
               double dx = Math.sin(t * 0.8 + i * 1.7) * (1.7 + i * 0.25);
               double dy = Math.sin(t * 1.1 + i * 2.1) * 1.4;
               double dz = Math.cos(t * 0.6 + i * 1.3) * (1.4 + i * 0.20);
               spheres.get(i).center = new Vec3(base.x + dx, base.y + dy, base.z + dz);
            }

            // ---------- рендер ----------
            renderToBuffer(pixels);
            image.getPixelWriter().setPixels(
                    0, 0, WIDTH, HEIGHT,
                    PixelFormat.getIntArgbInstance(),
                    pixels, 0, WIDTH);
            gc.drawImage(image, 0, 0, WIDTH * SCALE, HEIGHT * SCALE);

            // FPS в правом верхнем углу
            String fpsText = String.format("FPS: %.1f", fps);
            gc.setFill(Color.WHITE);
            gc.fillText(fpsText, WIDTH * SCALE - 100, 24);
         }
      };
      timer.start();
   }

   public static void main(String[] args) {
      launch(args);
   }
}