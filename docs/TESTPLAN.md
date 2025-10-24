**TEST PLAN – Virtual Temperature Sensor (simtemp)**
## Verification Evidence

All functional tests were covered in the demo video:

| Test | Description                                                           | Result |
|------|-----------------------------------------------------------------------|--------|
| T1   | Load/Unload (`insmod`/`rmmod`)                                        | PASS   |
| T2   | Periodic Read (`--follow`)                                            | PASS   |
| T3   | Threshold Event (`--test`)                                            | PASS   |
| T4   | Error Paths (invalid sysfs write)                                     | PASS   |
| T5   | Concurrency (2 instances of CLI with continuous reads)                | PASS   |
| T5   | Writing sysfs parameters while continuous reading at fast sample rate | PASS   |
| T6   | API Contract (16 B struct read)                                       | PASS   |
