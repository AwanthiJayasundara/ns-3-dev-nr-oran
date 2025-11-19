# ns-3-dev-nr-oran

A fork of [`nsnam/ns-3-dev-git`](https://gitlab.com/nsnam/ns-3-dev) with two additional modules already integrated:

- **5G-LENA NR module** (from the [CTTC 5G-LENA project](https://5g-lena.cttc.es/)) vendored into `src/nr`
- **ns3-oran ORAN module** (from [NIST ns3-oran](https://github.com/usnistgov/ns3-oran)) vendored into `contrib/oran`

This repository is used for experiments that combine **5G NR** and **O-RAN** in ns-3, using a single tree that already contains both modules.

> ⚠️ This is **not** the official read-only mirror of ns-3. It is a personal fork with extra modules and custom changes.

# Table of Contents

* [Project Overview](#project-overview)  
  * [Repository Layout](#repository-layout)  
* [Model Description](#model-description)  
* [Features](#features)  
* [Minimum Requirements](#minimum-requirements)  
* [Getting Started](#getting-started)  
  * [Clone](#clone)  
  * [Configure & Build](#configure--build)  
* [Running the Examples](#running-the-examples)  
  * [NR (5G-LENA) Examples](#nr-5g-lena-examples)  
  * [ORAN Examples](#oran-examples)  
* [Updating NR and ORAN from my forks](#updating-nr-and-oran-from-my-forks)  
  * [Updating NR (`src/nr`)](#updating-nr-srcnr)  
  * [Updating ORAN (`contrib/oran`)](#updating-oran-contriboran)  
* [Documentation](#documentation)  
* [Notes & Limitations](#notes--limitations)  
* [Upstream References](#upstream-references)  

---

# Project Overview

This project combines:

- The **ns-3 dev** tree (`ns-3-dev-git`)
- The **5G-LENA NR** module (NR RAN model)
- The **ns3-oran** module (O-RAN-like RIC, reporting modules, logic modules / xApps)

into a **single repository** for easier development and experimentation. Instead of cloning and attaching NR and ORAN manually for every ns-3 tree, everything is already wired together here.

The goal is to have a convenient environment where:

- 5G NR radio access (CTTC 5G-LENA) can be simulated.
- O-RAN-inspired control using ns3-oran’s Near-RT RIC, reporting modules and xApps can be integrated.
- Custom scenarios and research code can be developed on top of both 5G Nr and ORAN.

## Repository Layout

Key directories in this fork:

- `src/nr/`  
  5G-LENA NR module (5G New Radio), integrated as a normal ns-3 module under `src`.

- `contrib/oran/`  
  ns3-oran ORAN module, implementing an O-RAN-like Near-RT RIC, reporting modules, logic modules (xApps) and a data repository (SQLite-based).

- `src/lte/`  
  The standard ns-3 LTE module from upstream.

- `examples/`, `src/*/examples/`, `contrib/oran/examples/`  
  Example simulation programs for ns-3 core, NR, and ORAN.

- `scratch/`  
  Custom scenarios and quick experiments for this fork.

Both `src/nr` and `contrib/oran` are **vendored**: their original `.git` directories were removed, and the code is tracked directly as part of this repository.

---

# Model Description

**NR (5G-LENA)**

The NR module models 5G New Radio according to 3GPP specifications. It includes:

- 5G NR gNB and UE models
- Flexible numerologies, bandwidth parts, and schedulers
- Channel models, EPC integration, examples and tests

It is designed as a pluggable ns-3 module to simulate 5G NR networks with high fidelity.

**ORAN (ns3-oran)**

The ORAN module provides:

- A Near-RT RIC model
- Reporting modules that attach to ns-3 nodes and send measurements to the RIC
- Logic Modules (LMs) that act like O-RAN xApps
- A data repository (SQLite backend) for storing reports, commands and logs

It focuses on giving researchers infrastructure and data access so they can implement custom control logic (e.g. handover policies, ML-based decisions) without modifying core network models.

---

# Features

This combined repository allows you to:

- Build and run ns-3 dev with:
  - 5G-LENA NR (`src/nr`)
  - ns3-oran (`contrib/oran`)
- Run **NR-only** simulations (5G NR RAN)
- Run **ORAN-only** simulations (e.g., LTE-based handover controlled by RIC)
- Develop **combined NR + ORAN** scenarios
- Maintain local modifications to NR and ORAN via your own forks

---

# Minimum Requirements
* ns-3.42
* SQLite 3.7.17

## Optional Dependencies
* ONNX Runtime 1.14.1
* PyTorch 2.2.2

