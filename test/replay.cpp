#include "putt_detector.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char **argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: replay <clip.csv>\n"); return 2; }
  FILE *fp = std::fopen(argv[1], "r");
  if (!fp) { std::perror("open"); return 2; }

  PuttDetector det{DetectorConfig{}};
  char line[256];
  int detections = 0, rejects = 0, rows = 0;
  while (std::fgets(line, sizeof(line), fp)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    RawSample r{};
    int gx, gy, gz, ax, ay, az; unsigned long t;
    if (std::sscanf(line, "%lu,%d,%d,%d,%d,%d,%d",
                    &t, &gx, &gy, &gz, &ax, &ay, &az) != 7) continue;
    r.tUs = (uint32_t)t;
    r.gx=gx; r.gy=gy; r.gz=gz; r.ax=ax; r.ay=ay; r.az=az;
    ++rows;
    PuttEvent e = det.update(r);
    if (!e.detected) continue;
    if (e.decision == PuttDecision::Accept) {
      ++detections;
      std::printf("ACCEPT t=%u dur=%ums axis=%.2f rev=%d fwd=%.1f bell=%.2f "
                  "lin/ang=%.2f jerk=%.2f\n",
                  r.tUs, e.features.durationMs, e.features.axisConsistency,
                  e.features.reversalCount, e.features.peakForwardDps,
                  e.features.bellRatio, e.features.linAngRatio, e.features.impactJerk);
    } else {
      ++rejects;
      std::printf("REJECT(%s) t=%u dur=%ums axis=%.2f rev=%d lin/ang=%.2f\n",
                  e.reason, r.tUs, e.features.durationMs, e.features.axisConsistency,
                  e.features.reversalCount, e.features.linAngRatio);
    }
  }
  std::fclose(fp);
  std::printf("# %s: %d rows, %d accepted, %d rejected\n", argv[1], rows, detections, rejects);
  return 0;
}
