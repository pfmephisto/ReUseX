// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Spinner } from 'reusex-gui';

/** Labelled, for a route load. */
export const Labelled = () => <Spinner label="Loading project summary…" />;

/** Labelled with a more specific, stage-shaped subject. */
export const LoadingClouds = () => <Spinner label="Loading point clouds…" />;

/** Unlabelled — decorative ring only, announces nothing to a screen reader. */
export const Unlabelled = () => <Spinner />;
