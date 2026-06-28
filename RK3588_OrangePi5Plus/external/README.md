# External Dependencies

Vendored dependencies used by the experimental `sensor_ekf2` target:

- `PX4-ECL`: `46dd05a9159817035dab6acebc33f8a3da69d3a7`
- `Matrix`: `f981cea2aebfc9cfd930dce73ba6d4d6681e99c1`

The main Makefile wraps the CMake build, so normal usage is:

```bash
make ekf2
make run-ekf2
```

The Betaflight-style `sensor_bfnav` target does not vendor the full Betaflight tree. Its estimator files in `src/bfnav/` are adapted from Betaflight GPLv3 code at commit `25a7f7417164d88c0db833db00af2066857e1258`.
