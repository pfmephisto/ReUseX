// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Decodes sensor frames OFF the GUI thread, with a cache. The frame browser
// asks for a frame (colour, depth, confidence, labels) or a thumbnail and
// gets a signal when it is ready; scrubbing a slider never blocks the window.
//
// Two worker threads, each with its OWN read-only ProjectDB on the project
// file (ProjectDB is not thread-safe; WAL lets readers run beside the GUI's
// connection): one for full frames, one for filmstrip thumbnails, so a long
// strip never delays the A/B images. Both serve the NEWEST request first and
// drop requests nobody can see any more (a fast scrub queues dozens).
//
// The decoded planes are QImages the GUI paints directly. Depth, confidence
// and labels are Indexed8 with 0 = no data, so their colours come from the
// theme at paint time (a theme switch needs no re-decode):
//   depth       1..255, near = 255
//   confidence  1..3 (ARKit low/medium/high) when the stored values are
//               0..2, else 1..255 like depth
//   labels      1 + palette slot (class id mod --label-count)

#include <QHash>
#include <QImage>
#include <QObject>
#include <QString>

#include <memory>
#include <utility>
#include <vector>

namespace rux::qt {

struct DecodedFrame {
  int id = -1;
  QImage color; ///< RGB888, empty when the frame has none
  QImage depth; ///< Indexed8, see the header
  double depth_min_m = 0, depth_max_m = 0;
  QImage confidence;              ///< Indexed8
  bool confidence_levels = false; ///< 1..3 = low/medium/high
  QImage labels;                  ///< Indexed8, empty without segmentation
  /// Pixels per class id in the segmentation, sorted by count, largest first.
  std::vector<std::pair<int, int>> label_pixels;
  int label_total = 0; ///< labelled pixels
  QString error;       ///< why the frame could not be read, if it could not

  qint64 bytes() const;
};

class FrameImageLoader : public QObject {
  Q_OBJECT
    public:
  explicit FrameImageLoader(QObject *parent = nullptr);
  ~FrameImageLoader() override;

  /// Point the workers at a project file (empty = none). Clears the cache.
  void set_project(const QString &path);
  /// The palette size labels are folded into (--label-count).
  void set_label_slots(int slots);

  /// A decoded frame from the cache, or nullptr (then request() it).
  /// A hit counts as a use for the LRU.
  std::shared_ptr<const DecodedFrame> frame(int id) const;
  void request(int id);

  /// A cached thumbnail, or a null image (then request_thumbnail() it).
  QImage thumbnail(int id) const;
  void request_thumbnail(int id);
  /// Thumbnail edge length (the longer side), in device pixels.
  void set_thumbnail_size(int px);

  /// Requests queued or decoding, across every loader in the process. The
  /// gallery waits for 0 before it screenshots.
  static int busy();

    signals:
  void frame_ready(int id);
  void thumbnail_ready(int id);

    private:
  struct Worker;
  void deliver_frame(std::shared_ptr<DecodedFrame> f, unsigned generation);
  void deliver_thumbnail(int id, QImage img, unsigned generation);

  std::unique_ptr<Worker> frames_;
  std::unique_ptr<Worker> thumbs_;
  QHash<int, std::shared_ptr<const DecodedFrame>> cache_;
  mutable std::vector<int> lru_; ///< most recently used last
  qint64 cache_bytes_ = 0;
  QHash<int, QImage> thumb_cache_;
  QString path_;
  int label_slots_ = 8;
  int thumb_px_ = 128;
  unsigned generation_ = 0;
};

} // namespace rux::qt
