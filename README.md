# sentinelnet

Byzantine-fault-tolerant 5-node sensor mesh for the ESP32-C6, with a desktop simulation harness.

Each node samples sensors, authenticates messages with HMAC-SHA256, and participates in a BFT consensus round using online softmax voting. CUSUM detects anomalies in the fused sensor stream. Up to 1 Byzantine node is tolerated in a 5-node mesh.

```
# Desktop simulation
cd sim && cmake -B build && cmake --build build && ./build/sentinelnet_sim

# ESP32-C6 firmware
idf.py build flash monitor
```

All 16 tests pass on the simulation harness.
