# Offline Three.js runtime

Unmodified files from the official npm `three@0.128.0` package (MIT; see LICENSE).
Retains the viewer's existing r128 API; this is not a library-version upgrade.

Source: https://registry.npmjs.org/three/-/three-0.128.0.tgz

Package integrity (verified before extraction):
`sha512-i0ap/E+OaSfzw7bD1TtYnPo3VEplkl70WX5fZqZnfZsE3k3aSFudqrrC9ldFZfYFkn1zwDmBcdGfiIm/hnbyZA==`

Files:

- `build/three.min.js` -> `three.min.js`
- `examples/js/controls/OrbitControls.js` -> `OrbitControls.js`
- `LICENSE` -> `LICENSE`

Load three.min.js before OrbitControls.js and the application viewer scripts.
These are required runtime assets, not temporary downloads. Keep them in source
control and include them in every offline publish alongside their license.
