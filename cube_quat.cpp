#include <GL/glut.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <array>

// ---------- Quaternion utilities ----------
struct Quat {
  // We'll store as (w, x, y, z)
  double w{1}, x{0}, y{0}, z{0};
};

static Quat normalize(const Quat& q) {
  double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n <= 0) return {1,0,0,0};
  return {q.w/n, q.x/n, q.y/n, q.z/n};
}

static Quat mul(const Quat& a, const Quat& b) {
  // Hamilton product: a*b
  return {
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
  };
}

static Quat fromAxisAngle(double ax, double ay, double az, double angle_rad) {
  // Axis must be non-zero; we normalize it.
  double n = std::sqrt(ax*ax + ay*ay + az*az);
  if (n <= 1e-12) return {1,0,0,0};
  ax /= n; ay /= n; az /= n;
  double s = std::sin(angle_rad * 0.5);
  return normalize({std::cos(angle_rad * 0.5), ax*s, ay*s, az*s});
}

static std::array<float, 16> quatToMat4(const Quat& q_in) {
  // OpenGL column-major 4x4 rotation matrix
  Quat q = normalize(q_in);
  const double w = q.w, x = q.x, y = q.y, z = q.z;

  const double xx = x*x, yy = y*y, zz = z*z;
  const double xy = x*y, xz = x*z, yz = y*z;
  const double wx = w*x, wy = w*y, wz = w*z;

  // 3x3 rotation
  double r00 = 1.0 - 2.0*(yy + zz);
  double r01 = 2.0*(xy - wz);
  double r02 = 2.0*(xz + wy);

  double r10 = 2.0*(xy + wz);
  double r11 = 1.0 - 2.0*(xx + zz);
  double r12 = 2.0*(yz - wx);

  double r20 = 2.0*(xz - wy);
  double r21 = 2.0*(yz + wx);
  double r22 = 1.0 - 2.0*(xx + yy);

  // Column-major for OpenGL: m[col*4 + row]
  std::array<float,16> m{};
  m[0]  = (float)r00; m[4]  = (float)r01; m[8]  = (float)r02; m[12] = 0.0f;
  m[1]  = (float)r10; m[5]  = (float)r11; m[9]  = (float)r12; m[13] = 0.0f;
  m[2]  = (float)r20; m[6]  = (float)r21; m[10] = (float)r22; m[14] = 0.0f;
  m[3]  = 0.0f;       m[7]  = 0.0f;       m[11] = 0.0f;       m[15] = 1.0f;
  return m;
}

// ---------- Visualization ----------
static Quat g_q;                // current orientation
static float g_rot_step = 2.0f; // degrees per keypress
static int g_width = 900, g_height = 600;

static void drawAxes(float len=1.2f) {
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  // X (red)
  glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(len,0,0);
  // Y (green)
  glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,len,0);
  // Z (blue)
  glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,len);
  glEnd();
}

static void drawCubeWire(float s=1.0f) {
  glColor3f(1,1,1);
  glutWireCube(s);
}

static void drawCubeSolid(float s=1.0f) {
  glColor3f(0.7f, 0.7f, 0.9f);
  glutSolidCube(s);
}

static void printQuat() {
  Quat q = normalize(g_q);
  std::cout << "q = [w x y z] = "
            << q.w << " " << q.x << " " << q.y << " " << q.z << "\n";
}

static void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Camera
  gluLookAt(0, 0, 3.0,  // eye
            0, 0, 0,    // center
            0, 1, 0);   // up

  // World axes (fixed)
  drawAxes();

  // Apply quaternion rotation to cube
  auto M = quatToMat4(g_q);
  glPushMatrix();
  glMultMatrixf(M.data());

  // Cube axes (rotating with cube)
  drawAxes(0.9f);

  // Cube
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.0f, 1.0f);
  drawCubeSolid(1.0f);
  glDisable(GL_POLYGON_OFFSET_FILL);

  drawCubeWire(1.01f);
  glPopMatrix();

  glutSwapBuffers();
}

static void reshape(int w, int h) {
  g_width = w; g_height = h;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (h>0)? (double)w/(double)h : 1.0, 0.1, 100.0);
  glMatrixMode(GL_MODELVIEW);
}

static void applyIncrementalRotation(double ax, double ay, double az, double deg) {
  double rad = deg * M_PI / 180.0;
  Quat dq = fromAxisAngle(ax, ay, az, rad);
  // Pre-multiply: dq * q  (world axes)
  g_q = normalize(mul(dq, g_q));
  printQuat();
  glutPostRedisplay();
}

static void keyboard(unsigned char key, int, int) {
  switch (key) {
    case 27: // ESC
    case 'q':
      std::exit(0);
    case 'r': // reset
      g_q = {1,0,0,0};
      printQuat();
      glutPostRedisplay();
      break;

    // Roll around Z
    case 'e': applyIncrementalRotation(0,0,1, +g_rot_step); break;
    case 'c': applyIncrementalRotation(0,0,1, -g_rot_step); break;

    // Pitch around X
    case 'w': applyIncrementalRotation(1,0,0, +g_rot_step); break;
    case 's': applyIncrementalRotation(1,0,0, -g_rot_step); break;

    // Yaw around Y
    case 'a': applyIncrementalRotation(0,1,0, +g_rot_step); break;
    case 'd': applyIncrementalRotation(0,1,0, -g_rot_step); break;

    case '+': g_rot_step = std::min(20.0f, g_rot_step + 0.5f); break;
    case '-': g_rot_step = std::max(0.5f,  g_rot_step - 0.5f); break;
    default: break;
  }
}

static void initGL() {
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
}

int main(int argc, char** argv) {
  // Optional: parse initial quaternion from args: w x y z
  if (argc == 5) {
    g_q.w = std::atof(argv[1]);
    g_q.x = std::atof(argv[2]);
    g_q.y = std::atof(argv[3]);
    g_q.z = std::atof(argv[4]);
    g_q = normalize(g_q);
  } else {
    g_q = {1,0,0,0};
  }

  std::cout << "Cube Quaternion Viewer\n"
            << "Controls:\n"
            << "  w/s: pitch +/- (X axis)\n"
            << "  a/d: yaw   +/- (Y axis)\n"
            << "  e/c: roll  +/- (Z axis)\n"
            << "  r: reset,  +/-: change step,  q or ESC: quit\n";
  printQuat();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(g_width, g_height);
  glutCreateWindow("Quaternion Cube Orientation (C++ / OpenGL)");

  initGL();
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);

  glutMainLoop();
  return 0;
}
