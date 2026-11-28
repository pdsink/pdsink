Subset of USB PD CTS applicable to this project (test names, grouping,
enumeration).

- Version used: USB PD CTS Q3 2026

## Notes

- `TEST.PD.EPR.SNK3` tests
  - The Tester is not a full PD stack. Since UUT enters EPR Mode right after the
    SPR contract, some tests reorder the CTS steps. See comments in the code.
- `TEST.PD.PROT.SNK3.5` Sink PPS Normal Operation
  - `COMMON.PROC.BU.5` sends 5 PDOs with the PPS APDO at position 5, but step 4a
    checks RDO Object Position = 2. Most likely a typo in CTS.
