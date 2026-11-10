# Forest false-trunk fixture

`forest_false_trunk.npz` contains a real Gazebo `v2_forest` Husky LiDAR scan
captured at the previous acquisition position (world x=92.66, y=-64.996,
yaw=4.17723). The request used a displaced aerial estimate of the nearby pine.
Only points within 8 m horizontally are retained; this preserves the detector's
ground-fit and target-crop inputs.

The previous locator accepted two foliage rings at 1.051 m and 1.206 m above
ground as an approximately 0.88 m trunk. Their centre was about 1.15 m from the
actual pine stem. The real lower stem had only one visible valid ring, so this
view should remain unresolved.

The fixture stores XYZ, the requested centre in the scan frame and its search
tolerance. The regression verifies that foliage without a supporting lower
stem is rejected. Ground truth is used only to evaluate the capture, never as
an input to the production detector.

Original capture:
`.mission-control/dbh-cause-retest-20261105/acquisition/clouds/1793875564016770760_locate/scan_02.npz`

# Forest overhead-ground-strip fixture

`forest_overhead_strip.npz` stores an actual retained overhead return and the
segmented ground samples within 1.5 m of its XY location at simulation time
238.77 s in `navigation-root-cause-20261106/instrumented3/capture`

The return is 14.235 m from Husky and approximately 2.702 m above the simulator's
terrain collision mesh. The 0.75 m search has no ground support. After grouping,
the 1.5 m search has nine ground samples but fails the horizontal-spread check
The regression verifies that the bounded-height fallback rejects this return
while disabling the fallback reproduces the original decision

Ground-truth clearance is a test assertion only and is never supplied to the filter
