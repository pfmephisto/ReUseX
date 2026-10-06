// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The Segmentering view's image, front and centre: the colour frame fitted to
 * the stage (aspect kept), the decoded label mask on a canvas above it, the
 * prompts drawn as positioned boxes/points, and a pointer layer on top —
 * drag = box prompt, click = point prompt.
 *
 * Everything is placed in percentages of the image, so prompts (stored in
 * image pixels) stay put through zoom-to-fit and resizes, and the only canvas
 * is the mask, drawn once per mask/selection at the mask's own resolution.
 */

import { useEffect, useRef, useState, type CSSProperties, type PointerEvent } from 'react';

import type { LabelImage } from '../../data/labelPng';
import { cornersToBox, maskRgba, pointBox, type ImageBox, type SegPrompt } from '../../data/segmentView';
import { parseColor, readLabelPalette } from '../../viewport/labelColors';
import styles from './SegmentStage.module.css';

/** Below this many display pixels of movement a press is a click (a point prompt). */
const CLICK_SLOP = 4;
const MASK_ALPHA = { normal: 140, strong: 210, faint: 60 };

export interface SegmentStageProps {
  imageUrl: string;
  alt: string;
  /** Natural size of the colour image, once loaded. */
  size: { width: number; height: number } | null;
  onLoad: (size: { width: number; height: number }) => void;
  prompts: readonly SegPrompt[];
  /** Palette slot (prompt index) of a prompt, or null when it would not be sent. */
  slotOf: (promptId: string) => number | null;
  paletteSize: number;
  /** A drawn box (`at` null) or a click: its marker box and the exact clicked pixel. */
  onDraw: (box: ImageBox, at: [number, number] | null) => void;
  /** Touch draw mode: a finger drag draws a box instead of scrolling the page. */
  touchDraw?: boolean;
  mask: LabelImage | null;
  showMask: boolean;
  /** Highlighted prompt index in the mask, or null. */
  selected: number | null;
  disabled: boolean;
}

interface Drag {
  pointer: number;
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

const pct = (value: number, of: number) => `${(100 * value) / of}%`;

export function SegmentStage({
  imageUrl,
  alt,
  size,
  onLoad,
  prompts,
  slotOf,
  paletteSize,
  onDraw,
  mask,
  showMask,
  selected,
  disabled,
  touchDraw = false,
}: SegmentStageProps) {
  const maskRef = useRef<HTMLCanvasElement>(null);
  const layerRef = useRef<HTMLDivElement>(null);
  const [drag, setDrag] = useState<Drag | null>(null);

  useEffect(() => {
    const canvas = maskRef.current;
    if (!canvas || !mask) return;
    canvas.width = mask.width;
    canvas.height = mask.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const palette = readLabelPalette(canvas);
    const pixels = maskRgba(mask, palette.colors.map(parseColor), selected, MASK_ALPHA);
    ctx.putImageData(new ImageData(pixels, mask.width, mask.height), 0, 0);
  }, [mask, selected]);

  const toLocal = (e: PointerEvent<HTMLDivElement>) => {
    const rect = e.currentTarget.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top, w: rect.width, h: rect.height };
  };

  const onPointerDown = (e: PointerEvent<HTMLDivElement>) => {
    if (disabled || !size || e.button !== 0) return;
    const p = toLocal(e);
    e.currentTarget.setPointerCapture(e.pointerId);
    setDrag({ pointer: e.pointerId, x0: p.x, y0: p.y, x1: p.x, y1: p.y });
  };

  const onPointerMove = (e: PointerEvent<HTMLDivElement>) => {
    if (!drag || drag.pointer !== e.pointerId) return;
    const p = toLocal(e);
    setDrag({ ...drag, x1: p.x, y1: p.y });
  };

  const onPointerUp = (e: PointerEvent<HTMLDivElement>) => {
    if (!drag || drag.pointer !== e.pointerId || !size) return;
    const p = toLocal(e);
    setDrag(null);
    const sx = size.width / p.w;
    const sy = size.height / p.h;
    const moved = Math.abs(p.x - drag.x0) >= CLICK_SLOP || Math.abs(p.y - drag.y0) >= CLICK_SLOP;
    if (!moved) {
      const at: [number, number] = [
        Math.min(size.width - 1, Math.max(0, drag.x0 * sx)),
        Math.min(size.height - 1, Math.max(0, drag.y0 * sy)),
      ];
      onDraw(pointBox(at[0], at[1], size.width, size.height), at);
    } else {
      onDraw(
        cornersToBox({ x: drag.x0 * sx, y: drag.y0 * sy }, { x: p.x * sx, y: p.y * sy }, size.width, size.height),
        null,
      );
    }
  };

  const frameStyle = size ? ({ '--ar': `${size.width} / ${size.height}` } as CSSProperties) : undefined;

  return (
    <div className={styles.stage}>
      <div className={`${styles.frame} ${size ? '' : styles.loading}`} style={frameStyle}>
        <img
          className={styles.image}
          src={imageUrl}
          alt={alt}
          draggable={false}
          onLoad={(e) => onLoad({ width: e.currentTarget.naturalWidth, height: e.currentTarget.naturalHeight })}
        />
        <canvas
          ref={maskRef}
          className={styles.mask}
          hidden={!mask || !showMask}
          aria-hidden="true"
        />
        {size &&
          prompts.map((p) => {
            if (!p.box) return null;
            const slot = slotOf(p.id);
            const colour = {
              '--c': slot === null ? 'var(--color-text-faint)' : `var(--label-${slot % paletteSize})`,
            } as CSSProperties;
            const [x1, y1, x2, y2] = p.box;
            const number = slot === null ? '·' : String(slot + 1);
            if (p.point) {
              const [cx, cy] = p.at ?? [(x1 + x2) / 2, (y1 + y2) / 2];
              return (
                <span
                  key={p.id}
                  className={styles.point}
                  style={{ ...colour, left: pct(cx, size.width), top: pct(cy, size.height) }}
                >
                  <span className={styles.tag}>{number}</span>
                </span>
              );
            }
            return (
              <span
                key={p.id}
                className={styles.box}
                style={{
                  ...colour,
                  left: pct(x1, size.width),
                  top: pct(y1, size.height),
                  width: pct(x2 - x1, size.width),
                  height: pct(y2 - y1, size.height),
                }}
              >
                <span className={styles.tag}>{number}</span>
              </span>
            );
          })}
        {drag && layerRef.current && (
          <span
            className={styles.dragBox}
            style={{
              left: `${Math.min(drag.x0, drag.x1)}px`,
              top: `${Math.min(drag.y0, drag.y1)}px`,
              width: `${Math.abs(drag.x1 - drag.x0)}px`,
              height: `${Math.abs(drag.y1 - drag.y0)}px`,
            }}
          />
        )}
        <div
          ref={layerRef}
          className={`${styles.layer} ${touchDraw ? styles.layerDraw : ''} ${disabled ? styles.layerDisabled : ''}`}
          aria-label="Tegn en boks eller klik et punkt for at tilføje en prompt"
          role="application"
          onPointerDown={onPointerDown}
          onPointerMove={onPointerMove}
          onPointerUp={onPointerUp}
          onPointerCancel={() => setDrag(null)}
        />
      </div>
    </div>
  );
}
