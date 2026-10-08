// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/FrameImageLoader.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <QMetaObject>
#include <QPointer>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <iterator>
#include <map>
#include <mutex>
#include <thread>

namespace rux::qt {
namespace {

std::atomic<int> g_busy{0};

/// Cache budget for decoded frames: a 720x960 frame with every plane is
/// about 3 MB, so this keeps the last few dozen A/B frames.
constexpr qint64 kFrameCacheBytes = 160LL * 1024 * 1024;
/// Thumbnails kept (a strip shows a few dozen; scrolling refills).
constexpr int kThumbCacheEntries = 1200;

QImage to_qimage(const cv::Mat &m) {
  if (m.empty())
    return {};
  cv::Mat rgb;
  if (m.type() == CV_8UC3)
    cv::cvtColor(m, rgb, cv::COLOR_BGR2RGB);
  else if (m.type() == CV_8UC4)
    cv::cvtColor(m, rgb, cv::COLOR_BGRA2RGB);
  else if (m.type() == CV_8UC1)
    cv::cvtColor(m, rgb, cv::COLOR_GRAY2RGB);
  else
    return {};
  QImage img(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
             QImage::Format_RGB888);
  return img.copy(); // own the pixels; rgb dies here
}

/// Stretch the valid (non-zero) values of @p m to 1..255, near/low = 255
/// when @p invert. Returns the valid range through @p lo / @p hi.
QImage stretch_indexed(const cv::Mat &m, bool invert, double &lo, double &hi) {
  lo = hi = 0;
  if (m.empty() || m.channels() != 1)
    return {};
  cv::Mat values;
  m.convertTo(values, CV_64F);
  const cv::Mat valid = values > 0.0;
  if (cv::countNonZero(valid) == 0)
    return {};
  cv::minMaxLoc(values, &lo, &hi, nullptr, nullptr, valid);
  QImage out(m.cols, m.rows, QImage::Format_Indexed8);
  out.setColorCount(256);
  const double span = hi > lo ? hi - lo : 1.0;
  for (int y = 0; y < m.rows; ++y) {
    const double *row = values.ptr<double>(y);
    uchar *o = out.scanLine(y);
    for (int x = 0; x < m.cols; ++x) {
      if (row[x] <= 0.0) {
        o[x] = 0;
        continue;
      }
      double t = (row[x] - lo) / span;
      if (invert)
        t = 1.0 - t;
      o[x] = static_cast<uchar>(1 + std::lround(t * 254.0));
    }
  }
  return out;
}

std::shared_ptr<DecodedFrame> decode_frame(const reusex::ProjectDB &db, int id,
                                           int nslots) {
  auto f = std::make_shared<DecodedFrame>();
  f->id = id;
  try {
    f->color = to_qimage(db.sensor_frame_image(id));

    const cv::Mat depth = db.sensor_frame_depth(id);
    double lo = 0, hi = 0;
    f->depth = stretch_indexed(depth, /*invert=*/true, lo, hi);
    f->depth_min_m = lo / 1000.0; // stored in millimetres
    f->depth_max_m = hi / 1000.0;

    const cv::Mat conf = db.sensor_frame_confidence(id);
    if (!conf.empty() && conf.channels() == 1) {
      double cmin = 0, cmax = 0;
      cv::minMaxLoc(conf, &cmin, &cmax);
      if (conf.depth() == CV_8U && cmax <= 2.0) {
        // ARKit confidence: 0 low, 1 medium, 2 high -> indices 1..3.
        f->confidence_levels = true;
        QImage out(conf.cols, conf.rows, QImage::Format_Indexed8);
        out.setColorCount(256);
        for (int y = 0; y < conf.rows; ++y) {
          const uchar *row = conf.ptr<uchar>(y);
          uchar *o = out.scanLine(y);
          for (int x = 0; x < conf.cols; ++x)
            o[x] = static_cast<uchar>(row[x] + 1);
        }
        f->confidence = out;
      } else {
        f->confidence = stretch_indexed(conf, /*invert=*/false, lo, hi);
      }
    }

    if (db.has_segmentation_image(id)) {
      const cv::Mat seg = db.segmentation_image(id); // CV_32S, -1 = none
      if (!seg.empty() && seg.type() == CV_32SC1) {
        QImage out(seg.cols, seg.rows, QImage::Format_Indexed8);
        out.setColorCount(256);
        std::map<int, int> counts;
        const int n = std::clamp(nslots, 1, 254);
        for (int y = 0; y < seg.rows; ++y) {
          const int *row = seg.ptr<int>(y);
          uchar *o = out.scanLine(y);
          for (int x = 0; x < seg.cols; ++x) {
            const int k = row[x];
            if (k < 0) {
              o[x] = 0;
              continue;
            }
            o[x] = static_cast<uchar>(1 + k % n);
            ++counts[k];
            ++f->label_total;
          }
        }
        f->labels = out;
        f->label_pixels.assign(counts.begin(), counts.end());
        std::stable_sort(
            f->label_pixels.begin(), f->label_pixels.end(),
            [](const auto &a, const auto &b) { return a.second > b.second; });
      }
    }
  } catch (const std::exception &e) {
    f->error = QString::fromUtf8(e.what());
  }
  return f;
}

QImage decode_thumbnail(const reusex::ProjectDB &db, int id, int px) {
  const cv::Mat m = db.sensor_frame_image(id);
  if (m.empty())
    return {};
  const double s =
      static_cast<double>(px) / static_cast<double>(std::max(m.cols, m.rows));
  cv::Mat small;
  cv::resize(m, small, cv::Size(), s, s, cv::INTER_AREA);
  return to_qimage(small);
}

} // namespace

qint64 DecodedFrame::bytes() const {
  return color.sizeInBytes() + depth.sizeInBytes() + confidence.sizeInBytes() +
         labels.sizeInBytes() + 256;
}

// ------------------------------------------------------------------ worker --

/// One decode thread with its own read-only connection. Requests are served
/// newest first; beyond `max_queue` the oldest are dropped.
struct FrameImageLoader::Worker {
  enum class Kind { frame, thumbnail };

  Worker(FrameImageLoader *owner, Kind kind, std::size_t max_queue)
      : owner_(owner), kind_(kind), max_queue_(max_queue) {
    thread_ = std::thread([this] { run(); });
  }

  ~Worker() {
    {
      std::lock_guard<std::mutex> lock(m_);
      stop_ = true;
      g_busy -= static_cast<int>(queue_.size());
      queue_.clear();
    }
    cv_.notify_all();
    thread_.join();
  }

  void configure(const QString &path, unsigned generation, int nslots,
                 int thumb_px) {
    std::lock_guard<std::mutex> lock(m_);
    path_ = path.toStdString();
    generation_ = generation;
    slots_ = nslots;
    thumb_px_ = thumb_px;
    reopen_ = true;
    g_busy -= static_cast<int>(queue_.size());
    queue_.clear();
  }

  void push(int id) {
    {
      std::lock_guard<std::mutex> lock(m_);
      if (path_.empty())
        return;
      const auto it = std::find(queue_.begin(), queue_.end(), id);
      if (it != queue_.end()) {
        queue_.erase(it); // re-queued as the newest
      } else {
        ++g_busy;
      }
      queue_.push_back(id);
      while (queue_.size() > max_queue_) {
        queue_.pop_front();
        --g_busy;
      }
    }
    cv_.notify_one();
  }

    private:
  void run() {
    std::unique_ptr<reusex::ProjectDB> db;
    std::string open_path;
    for (;;) {
      int id = 0;
      std::string path;
      unsigned generation = 0;
      int nslots = 8, px = 128;
      {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [this] { return stop_ || !queue_.empty() || reopen_; });
        if (stop_)
          return;
        if (reopen_) {
          reopen_ = false;
          db.reset(); // the old project's connection goes now
          open_path.clear();
        }
        if (queue_.empty())
          continue;
        id = queue_.back(); // newest first
        queue_.pop_back();
        path = path_;
        generation = generation_;
        nslots = slots_;
        px = thumb_px_;
      }

      std::shared_ptr<DecodedFrame> frame;
      QImage thumb;
      try {
        if (!db || open_path != path) {
          db = std::make_unique<reusex::ProjectDB>(path, /*readOnly=*/true);
          open_path = path;
        }
        if (kind_ == Kind::frame)
          frame = decode_frame(*db, id, nslots);
        else
          thumb = decode_thumbnail(*db, id, px);
      } catch (const std::exception &e) {
        if (kind_ == Kind::frame) {
          frame = std::make_shared<DecodedFrame>();
          frame->id = id;
          frame->error = QString::fromUtf8(e.what());
        }
      }

      // Deliver on the GUI thread; the owner joins this thread before it is
      // destroyed, and a delivery queued just before that is dropped with it.
      if (kind_ == Kind::frame)
        QMetaObject::invokeMethod(
            owner_,
            [o = owner_, frame, generation] {
              o->deliver_frame(frame, generation);
            },
            Qt::QueuedConnection);
      else
        QMetaObject::invokeMethod(
            owner_,
            [o = owner_, id, thumb, generation] {
              o->deliver_thumbnail(id, thumb, generation);
            },
            Qt::QueuedConnection);
      --g_busy;
    }
  }

  FrameImageLoader *owner_;
  Kind kind_;
  std::size_t max_queue_;
  std::thread thread_;
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<int> queue_;
  bool stop_ = false;
  bool reopen_ = false;
  std::string path_;
  unsigned generation_ = 0;
  int slots_ = 8;
  int thumb_px_ = 128;
};

// ------------------------------------------------------------------ loader --

FrameImageLoader::FrameImageLoader(QObject *parent)
    : QObject(parent),
      // A/B need at most the two newest; a scrub drops the rest.
      frames_(std::make_unique<Worker>(this, Worker::Kind::frame, 4)),
      // A strip shows a few dozen thumbnails at once.
      thumbs_(std::make_unique<Worker>(this, Worker::Kind::thumbnail, 96)) {}

FrameImageLoader::~FrameImageLoader() {
  // Join both threads before the QObject goes (their deliveries target it).
  frames_.reset();
  thumbs_.reset();
}

int FrameImageLoader::busy() { return g_busy.load(); }

void FrameImageLoader::set_project(const QString &path) {
  path_ = path;
  ++generation_;
  cache_.clear();
  lru_.clear();
  cache_bytes_ = 0;
  thumb_cache_.clear();
  frames_->configure(path, generation_, label_slots_, thumb_px_);
  thumbs_->configure(path, generation_, label_slots_, thumb_px_);
}

void FrameImageLoader::set_label_slots(int nslots) {
  if (nslots == label_slots_ || nslots <= 0)
    return;
  label_slots_ = nslots;
  set_project(path_); // labels are folded at decode time
}

void FrameImageLoader::set_thumbnail_size(int px) {
  if (px == thumb_px_ || px <= 0)
    return;
  thumb_px_ = px;
  thumb_cache_.clear();
  thumbs_->configure(path_, generation_, label_slots_, thumb_px_);
}

std::shared_ptr<const DecodedFrame> FrameImageLoader::frame(int id) const {
  auto f = cache_.value(id);
  if (f) {
    // Least recently USED: a frame shown for a while must outlive frames
    // scrubbed past since.
    const auto it = std::find(lru_.begin(), lru_.end(), id);
    if (it != lru_.end() && std::next(it) != lru_.end()) {
      lru_.erase(it);
      lru_.push_back(id);
    }
  }
  return f;
}

void FrameImageLoader::request(int id) {
  if (frame(id)) { // touches the LRU
    emit frame_ready(id);
    return;
  }
  frames_->push(id);
}

QImage FrameImageLoader::thumbnail(int id) const {
  return thumb_cache_.value(id);
}

void FrameImageLoader::request_thumbnail(int id) {
  if (!thumb_cache_.contains(id))
    thumbs_->push(id);
}

void FrameImageLoader::deliver_frame(std::shared_ptr<DecodedFrame> f,
                                     unsigned generation) {
  if (generation != generation_ || !f)
    return;
  const int id = f->id;
  if (!cache_.contains(id)) {
    cache_bytes_ += f->bytes();
    lru_.push_back(id);
  }
  cache_.insert(id, f);
  // Evict the least recently used beyond the budget (never the newest).
  while (cache_bytes_ > kFrameCacheBytes && lru_.size() > 2) {
    const int old = lru_.front();
    lru_.erase(lru_.begin());
    if (auto it = cache_.find(old); it != cache_.end()) {
      cache_bytes_ -= (*it)->bytes();
      cache_.erase(it);
    }
  }
  emit frame_ready(id);
}

void FrameImageLoader::deliver_thumbnail(int id, QImage img,
                                         unsigned generation) {
  if (generation != generation_)
    return;
  if (thumb_cache_.size() >= kThumbCacheEntries)
    thumb_cache_.clear(); // cheap and rare: the visible ones come back
  // A frame with no colour image gets an empty entry, so it is not asked
  // for again on every paint.
  thumb_cache_.insert(id,
                      img.isNull() ? QImage(1, 1, QImage::Format_RGB888) : img);
  emit thumbnail_ready(id);
}

} // namespace rux::qt
