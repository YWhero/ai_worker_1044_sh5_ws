The HX5 simulation profile mounts the official workspace `robotis_interfaces`
package here read-only. Its `msg/HandPressures.msg` and `msg/TactileSensor.msg`
take precedence over network downloads and match the running simulator.

This directory also preserves the mount point inside the read-only SDK mount.
