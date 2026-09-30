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
import javafx.stage.Stage;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class TinyRayTracerFX extends Application {

   static final int WIDTH      = 640;
   static final int HEIGHT     = 480;
   static final int SCALE      = 1;
   static final int MAX_DEPTH  = 8;
   static final double FOV     = Math.PI / 2.0;

   // Глубина, на которой «висит» сфера, управляемая мышью
   static final double MOUSE_SPHERE_Z = -16.0;

   // Индекс сферы, которой управляем мышью
   static final int MOUSE_SPHERE_INDEX = 0;

   // --- FPS ---
   private static final int FPS_WINDOW = 30;       // усреднять по 30 кадрам
   private final long[] frameTimes = new long[FPS_WINDOW];
   private int    frameIndex = 0;
   private int    framesFilled = 0;
   private double fps = 0;

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
      double norm() { return Math.sqrt(dot(this)); }
      Vec3 normalize() { double n = norm(); return n == 0 ? new Vec3() : mul(1.0 / n); }
   }

   static Vec3 reflect(Vec3 i, Vec3 n) { return i.sub(n.mul(2.0 * i.dot(n))); }

   static Vec3 refract(Vec3 i, Vec3 n, double eta) {
      double cosi = -Math.max(-1.0, Math.min(1.0, i.dot(n)));
      double etai = 1.0, etat = eta;
      Vec3 nn = n;
      if (cosi < 0) { cosi = -cosi; double t = etai; etai = etat; etat = t; nn = n.neg(); }
      double r = etai / etat;
      double k = 1.0 - r * r * (1.0 - cosi * cosi);
      if (k < 0) return new Vec3();
      return i.mul(r).add(nn.mul(r * cosi - Math.sqrt(k)));
   }

   // ==================== Material ====================
   static final class Material {
      final double refractiveIndex;
      final double[] albedo;
      Vec3 diffuseColor;
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
      final Material material;
      Sphere(Vec3 c, double r, Material m) { center = c; radius = r; material = m; }
      boolean rayIntersect(Vec3 orig, Vec3 dir, double[] outT) {
         Vec3   L   = center.sub(orig);
         double tca = L.dot(dir);
         double d2  = L.dot(L) - tca * tca;
         double r2  = radius * radius;
         if (d2 > r2) return false;
         double thc = Math.sqrt(r2 - d2);
         double t0  = tca - thc;
         double t1  = tca + thc;
         if (t0 < 0) t0 = t1;
         if (t0 < 0) return false;
         outT[0] = t0;
         return true;
      }
   }

   static final class Light {
      final Vec3 position;
      final double intensity;
      Light(Vec3 p, double i) { position = p; intensity = i; }
   }

   static final class Hit {
      Vec3     point    = new Vec3();
      Vec3     normal   = new Vec3();
      Material material = new Material();
   }

   // ==================== Сцена ====================
   final List<Sphere> spheres = new ArrayList<>();
   final List<Light>  lights  = new ArrayList<>();
   Vec3[] basePositions;

   boolean sceneIntersect(Vec3 orig, Vec3 dir, Hit hit) {
      double spheresDist = Double.MAX_VALUE;
      for (Sphere s : spheres) {
         double[] t = new double[1];
         if (s.rayIntersect(orig, dir, t) && t[0] < spheresDist) {
            spheresDist  = t[0];
            hit.point    = orig.add(dir.mul(t[0]));
            hit.normal   = hit.point.sub(s.center).normalize();
            hit.material = s.material;
         }
      }
      double checkerDist = Double.MAX_VALUE;
      if (Math.abs(dir.y) > 1e-3) {
         double d  = -(orig.y + 4.0) / dir.y;
         Vec3   pt = orig.add(dir.mul(d));
         if (d > 0 && Math.abs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < spheresDist) {
            checkerDist = d;
            hit.point   = pt;
            hit.normal  = new Vec3(0, 1, 0);
            Vec3 col = (((int)(0.5 * pt.x + 1000) + (int)(0.5 * pt.z)) & 1) != 0
                    ? new Vec3(1, 1, 1) : new Vec3(1, 0.7, 0.3);
            hit.material = new Material();
            hit.material.diffuseColor = col.mul(0.3);
         }
      }
      return Math.min(spheresDist, checkerDist) < 1000;
   }

   Vec3 castRay(Vec3 orig, Vec3 dir, int depth) {
      Hit hit = new Hit();
      if (depth > MAX_DEPTH || !sceneIntersect(orig, dir, hit)) {
         return new Vec3(0.2, 0.7, 0.8);
      }
      Vec3     point = hit.point;
      Vec3     N     = hit.normal;
      Material mat   = hit.material;

      Vec3 reflectDir  = reflect(dir, N).normalize();
      Vec3 refractDir  = refract(dir, N, mat.refractiveIndex).normalize();
      Vec3 reflectOrig = reflectDir.dot(N) < 0 ? point.sub(N.mul(1e-3)) : point.add(N.mul(1e-3));
      Vec3 refractOrig = refractDir.dot(N) < 0 ? point.sub(N.mul(1e-3)) : point.add(N.mul(1e-3));

      Vec3 reflectColor = castRay(reflectOrig, reflectDir, depth + 1);
      Vec3 refractColor = castRay(refractOrig, refractDir, depth + 1);

      double diffInt = 0, specInt = 0;
      for (Light light : lights) {
         Vec3   lightDir  = light.position.sub(point).normalize();
         double lightDist = light.position.sub(point).norm();

         Vec3 shadowOrig = lightDir.dot(N) < 0 ? point.sub(N.mul(1e-3)) : point.add(N.mul(1e-3));
         Hit sh = new Hit();
         if (sceneIntersect(shadowOrig, lightDir, sh)
                 && sh.point.sub(shadowOrig).norm() < lightDist) continue;

         diffInt += light.intensity * Math.max(0.0, lightDir.dot(N));
         specInt += Math.pow(
                 Math.max(0.0, -reflect(lightDir.neg(), N).dot(dir)),
                 mat.specularExponent) * light.intensity;
      }

      return mat.diffuseColor.mul(diffInt * mat.albedo[0])
              .add(new Vec3(1, 1, 1).mul(specInt * mat.albedo[1]))
              .add(reflectColor.mul(mat.albedo[2]))
              .add(refractColor.mul(mat.albedo[3]));
   }

   void renderToBuffer(int[] pixels) {
      IntStream.range(0, HEIGHT).parallel().forEach(j -> {
         for (int i = 0; i < WIDTH; i++) {
            double x =  (2 * (i + 0.5) / WIDTH  - 1) * Math.tan(FOV / 2) * WIDTH / (double) HEIGHT;
            double y = -(2 * (j + 0.5) / HEIGHT - 1) * Math.tan(FOV / 2);
            Vec3 dir = new Vec3(x, y, -1).normalize();
            Vec3 c   = castRay(new Vec3(0, 0, 0), dir, 0);

            double max = Math.max(c.x, Math.max(c.y, c.z));
            if (max > 1) c = c.mul(1.0 / max);

            int r = (int) (255 * clamp01(c.x));
            int g = (int) (255 * clamp01(c.y));
            int b = (int) (255 * clamp01(c.z));
            pixels[j * WIDTH + i] = 0xFF000000 | (r << 16) | (g << 8) | b;
         }
      });
   }

   static double clamp01(double v) { return Math.max(0.0, Math.min(1.0, v)); }

   // ==================== JavaFX ====================
   @Override
   public void start(Stage stage) {
      Material ivory     = new Material(1.0, new double[]{0.6, 0.3, 0.1, 0.0}, new Vec3(0.4, 0.4, 0.3),   50);
      Material glass     = new Material(1.5, new double[]{0.0, 0.5, 0.1, 0.8}, new Vec3(0.6, 0.7, 0.8),  125);
      Material redRubber = new Material(1.0, new double[]{0.9, 0.1, 0.0, 0.0}, new Vec3(0.3, 0.1, 0.1),   10);
      Material mirror    = new Material(1.0, new double[]{0.0,10.0, 0.8, 0.0}, new Vec3(1.0, 1.0, 1.0), 1425);

      spheres.add(new Sphere(new Vec3( 7.0,  5.0, -18), 4, mirror));      // ← индекс 0 — мышью
      spheres.add(new Sphere(new Vec3(-3.0,  0.0, -16), 2, ivory));       // ← анимируется
      spheres.add(new Sphere(new Vec3(-1.0, -1.5, -12), 2, glass));       // ← анимируется
      spheres.add(new Sphere(new Vec3( 1.5, -0.5, -18), 3, redRubber));   // ← анимируется

      lights.add(new Light(new Vec3(-20, 20,  20), 1.5));
      lights.add(new Light(new Vec3( 30, 50, -25), 1.8));
      lights.add(new Light(new Vec3( 30, 20,  30), 1.7));

      basePositions = new Vec3[spheres.size()];
      for (int i = 0; i < spheres.size(); i++) basePositions[i] = spheres.get(i).center;

      Canvas canvas = new Canvas(WIDTH * SCALE, HEIGHT * SCALE);
      GraphicsContext gc = canvas.getGraphicsContext2D();

      WritableImage image = new WritableImage(WIDTH, HEIGHT);
      int[] pixels = new int[WIDTH * HEIGHT];

      // ---------- Мышь ----------
      // Флаг «тащим ли сферу прямо сейчас» (только для UX; сфера и так следует за мышью)
      final boolean[] dragging = {false};

      // Преобразование координат курсора Canvas → мировые координаты на плоскости z = MOUSE_SPHERE_Z
      java.util.function.BiFunction<Double, Double, Vec3> screenToWorld = (mx, my) -> {
         double nx =  mx / (double) (WIDTH  * SCALE);   // 0..1 по Canvas
         double ny =  my / (double) (HEIGHT * SCALE);
         double tx =  (2 * nx - 1) * Math.tan(FOV / 2) * WIDTH / (double) HEIGHT;
         double ty = -(2 * ny - 1) * Math.tan(FOV / 2);
         // Масштабируем на глубину (расстояние от камеры до плоскости сферы)
         double depth = -MOUSE_SPHERE_Z;
         return new Vec3(tx * depth, ty * depth, MOUSE_SPHERE_Z);
      };

      canvas.setOnMousePressed(e -> {
         dragging[0] = true;
         Vec3 p = screenToWorld.apply(e.getX(), e.getY());
         spheres.get(MOUSE_SPHERE_INDEX).center = p;
      });
      canvas.setOnMouseDragged(e -> {
         if (!dragging[0]) return;
         Vec3 p = screenToWorld.apply(e.getX(), e.getY());
         spheres.get(MOUSE_SPHERE_INDEX).center = p;
      });
      canvas.setOnMouseReleased(e -> dragging[0] = false);

      StackPane root = new StackPane(canvas);
      stage.setScene(new Scene(root));
      stage.setTitle("Tiny Ray Tracer — JavaFX (1 сфера управляется мышью)");
      stage.setResizable(false);
      stage.show();

      // ---------- Анимация ----------
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
            for (int i = 0; i < spheres.size(); i++) {
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

            // FPS в правом верхнем углу с полупрозрачной плашкой для читаемости
            String fpsText = String.format("FPS: %.1f", fps);
            gc.setFont(javafx.scene.text.Font.font("Monospaced", 16));
            javafx.scene.text.Text measure =
                    new javafx.scene.text.Text(fpsText);
            measure.setFont(gc.getFont());
            double textW = measure.getLayoutBounds().getWidth();

            double padX = 8, padY = 6;
            double boxW = textW + 2 * padX;
            double boxH = 16 + 2 * padY;
            double boxX = WIDTH * SCALE - boxW - 10;
            double boxY = 10;

            gc.setFill( Color.WHITE );
            gc.fillText(fpsText, boxX + padX, boxY + padY + 13);
         }
      };
      timer.start();   }

   public static void main(String[] args) {
      launch(args);
   }
}