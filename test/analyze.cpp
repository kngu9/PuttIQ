// Diagnostic: run a recorded CSV clip through the real Preprocessor and report
// the stroke (gyro peak) and the impact transient (jerk peak) so we can judge
// whether impact is separable at the butt-end mount.
#include "putt_preprocess.h"
#include "vec3.h"
#include <cstdio>
#include <cstdlib>

int main(int argc, char **argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: analyze <clip.csv>\n"); return 2; }
  FILE *fp = std::fopen(argv[1], "r");
  if (!fp) { std::perror("open"); return 2; }

  Preprocessor pre{DetectorConfig{}};
  char line[256];
  DerivedSample prev{}; bool havePrev = false;

  float peakGyro = 0; uint32_t peakGyroT = 0;
  float peakJerk = 0; uint32_t peakJerkT = 0;
  float peakLin = 0;
  int validRows = 0;
  // jerk histogram of the largest few
  float top[8] = {0}; uint32_t topT[8] = {0};

  while (std::fgets(line, sizeof(line), fp)) {
    unsigned long t; int gx, gy, gz, ax, ay, az;
    if (std::sscanf(line, "%lu,%d,%d,%d,%d,%d,%d", &t, &gx, &gy, &gz, &ax, &ay, &az) != 7)
      continue;
    RawSample r{(uint32_t)t, (int16_t)gx, (int16_t)gy, (int16_t)gz,
                (int16_t)ax, (int16_t)ay, (int16_t)az};
    DerivedSample d = pre.process(r);
    if (!d.valid) { prev = d; havePrev = true; continue; }
    ++validRows;
    if (d.gyroDps > peakGyro) { peakGyro = d.gyroDps; peakGyroT = d.tUs; }
    if (d.linearMag > peakLin) peakLin = d.linearMag;
    if (havePrev && prev.valid) {
      float jerk = mag3(sub3(d.linearMps2, prev.linearMps2));
      if (jerk > peakJerk) { peakJerk = jerk; peakJerkT = d.tUs; }
      // track top-8 jerks
      for (int i = 0; i < 8; ++i) {
        if (jerk > top[i]) {
          for (int j = 7; j > i; --j) { top[j] = top[j-1]; topT[j] = topT[j-1]; }
          top[i] = jerk; topT[i] = d.tUs; break;
        }
      }
    }
    prev = d; havePrev = true;
  }
  std::fclose(fp);

  std::printf("valid rows: %d\n", validRows);
  std::printf("peak gyro : %.1f dps  @ t=%u\n", peakGyro, peakGyroT);
  std::printf("peak |lin|: %.2f m/s^2\n", peakLin);
  std::printf("peak jerk : %.2f m/s^2  @ t=%u  (%+d us from gyro peak)\n",
              peakJerk, peakJerkT, (int)((long)peakJerkT - (long)peakGyroT));
  std::printf("top jerks : ");
  for (int i = 0; i < 8; ++i) std::printf("%.2f ", top[i]);
  std::printf("\n\n");

  // Second pass: print an activity timeline (samples where gyro or jerk is notable).
  std::printf("  t_rel_ms   gyroDps   |lin|   jerk\n");
  fp = std::fopen(argv[1], "r");
  Preprocessor pre2{DetectorConfig{}};
  DerivedSample p2{}; bool hp2 = false; long t0 = -1;
  while (std::fgets(line, sizeof(line), fp)) {
    unsigned long t; int gx, gy, gz, ax, ay, az;
    if (std::sscanf(line, "%lu,%d,%d,%d,%d,%d,%d", &t, &gx, &gy, &gz, &ax, &ay, &az) != 7)
      continue;
    RawSample r{(uint32_t)t,(int16_t)gx,(int16_t)gy,(int16_t)gz,(int16_t)ax,(int16_t)ay,(int16_t)az};
    DerivedSample d = pre2.process(r);
    if (!d.valid) { p2 = d; hp2 = true; continue; }
    float jerk = (hp2 && p2.valid) ? mag3(sub3(d.linearMps2, p2.linearMps2)) : 0.0f;
    if (d.gyroDps > 8.0f || jerk > 3.0f) {
      if (t0 < 0) t0 = (long)d.tUs;
      std::printf("%9.1f  %8.1f  %6.2f  %6.2f\n",
                  ((long)d.tUs - t0) / 1000.0, d.gyroDps, d.linearMag, jerk);
    }
    p2 = d; hp2 = true;
  }
  std::fclose(fp);
  return 0;
}
