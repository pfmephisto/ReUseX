# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  buildNpmPackage,
  nodejs_22,
}:
buildNpmPackage {
  pname = "reusex-gui-frontend";
  version = "0.0.5";

  # Only the frontend tree, never the repo root. `default.nix` uses `src = ./.`,
  # so if this derivation did the same every C++ edit would change this input
  # hash and force a full npm build (and, via `default.nix`'s postInstall, the
  # other way round). The fileset is enumerated explicitly so that adding a
  # sibling directory under apps/rux/ cannot silently widen it.
  src = lib.fileset.toSource {
    root = ../../apps/rux/frontend;
    fileset = lib.fileset.unions [
      ../../apps/rux/frontend/index.html
      ../../apps/rux/frontend/package.json
      ../../apps/rux/frontend/package-lock.json
      ../../apps/rux/frontend/tsconfig.json
      ../../apps/rux/frontend/vite.config.ts
      ../../apps/rux/frontend/src
    ];
  };

  # Recompute after any package-lock.json change with:
  #   nix run nixpkgs#prefetch-npm-deps -- apps/rux/frontend/package-lock.json
  npmDepsHash = "sha256-yprC6TXTjIMQo32FbSc+D/t6Dy0VosQ4s6r9ZT0cjLs=";

  # Match the Node the lockfile was generated with; Vite 8 requires Node >= 20.
  nodejs = nodejs_22;

  # `npm run build` (the default npmBuildScript) is `tsc --noEmit && vite build`,
  # which emits a static bundle into dist/. There is nothing to `npm install`:
  # this is a web app, not a publishable node package, and the default install
  # phase would look for a `bin`/`main` entry point that does not exist.
  dontNpmInstall = true;

  # `rux gui` resolves its asset directory as: --assets, then $RUX_GUI_ASSETS,
  # then <install prefix>/share/reusex/gui (see apps/rux/src/gui/assets.cpp).
  # index.html must therefore sit at the root of that directory.
  installPhase = ''
    runHook preInstall

    mkdir -p $out/share/reusex/gui
    cp -r dist/. $out/share/reusex/gui/

    runHook postInstall
  '';

  meta = {
    description = "Static React/Vite bundle served by `rux gui` (ReUseX web frontend)";
    homepage = "https://github.com/pfmephisto/ReUseX";
    license = lib.licenses.gpl3Plus;
    maintainers = [];
    platforms = lib.platforms.all;
  };
}
