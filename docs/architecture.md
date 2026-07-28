# Architecture

Vulture-X uses a single asynchronous application boundary around separately
testable vehicle, target, estimator, safety, guidance, mission, and logging
modules. ArduPilot retains stabilization and flight-control authority.

Milestone 0 implements only configuration, shared types, logging, and process
bootstrap. Vehicle I/O and command authority are deliberately absent.

Future control outputs must flow through this ordering:

```text
target validation -> estimation -> guidance -> limiter chain
                  -> safety supervisor -> vehicle interface
```

The safety supervisor can reject guidance and request recovery. No downstream
component may override that decision.

