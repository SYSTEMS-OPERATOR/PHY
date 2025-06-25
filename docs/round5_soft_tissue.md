# Round 5 Soft Tissue Overview

This iteration introduces a rudimentary soft-tissue and bioelectric layer.  A
`MuscleSpec` dataclass describes attachment points and mechanical parameters.
`MuscleAgent` implements a very simple Hill-type force model.  Muscles are
updated by `ControlAgent` from EMG-like input signals.  A minimal
`WolffAdaptationEngine` and `EnergyAgent` provide bone density adaptation and
metabolic accounting.

These components are intentionally lightweight and are meant to serve as a
placeholder architecture for future high fidelity modelling.
