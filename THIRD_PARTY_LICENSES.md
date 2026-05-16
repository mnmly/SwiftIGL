# Third Party Licenses

SwiftIGL depends on the following third-party software, distributed as
part of `Frameworks/igl.xcframework` (header-only):

## libigl

**License:** Mozilla Public License 2.0 (MPL2)
**Website:** https://libigl.github.io
**Source:** https://github.com/libigl/libigl

The libigl headers shipped inside `igl.xcframework/macos-arm64/Headers/igl/`
remain under MPL2. A copy of the upstream license is bundled in
`igl.xcframework/macos-arm64/Headers/.licenses/libigl-LICENSE.MPL2`.

> This Source Code Form is subject to the terms of the Mozilla Public License,
> v. 2.0. If a copy of the MPL was not distributed with this file, you can
> obtain one at https://mozilla.org/MPL/2.0/.

The optional `copyleft/` and `restricted/` modules of libigl (GPL3 /
proprietary) are **not** included in this xcframework.

## Eigen

**License:** Mozilla Public License 2.0 (MPL2)
**Website:** https://eigen.tuxfamily.org
**Source:** https://gitlab.com/libeigen/eigen

Eigen 3.4.0 headers ship inside
`igl.xcframework/macos-arm64/Headers/{Eigen,unsupported}/`. The
xcframework is built with `EIGEN_MPL2_ONLY=1` to exclude Eigen's
optional LGPL components. Upstream license text is bundled in
`igl.xcframework/macos-arm64/Headers/.licenses/eigen-COPYING.MPL2`.

> This Source Code Form is subject to the terms of the Mozilla Public License,
> v. 2.0. If a copy of the MPL was not distributed with this file, you can
> obtain one at https://mozilla.org/MPL/2.0/.
