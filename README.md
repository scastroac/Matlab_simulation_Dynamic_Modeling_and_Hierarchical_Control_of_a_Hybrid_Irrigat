# Hybrid Irrigation System: Dynamic Modeling and Hierarchical Control

[![MATLAB](https://img.shields.io/badge/MATLAB-R2021a%2B-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Academic Concept](https://img.shields.io/badge/Context-MAPI4_Proceedings-success.svg)]()

This repository contains the official MATLAB simulation code for the paper: **"Dynamic Modeling and Hierarchical Control of a Hybrid Irrigation System for Shallow Soils in the Semi-Humid Andean Region"**. 

The script provides a rigorous numerical implementation of a non-linear, lumped-parameter hydrological model coupled with a 3-layer hierarchical control architecture. It is designed to simulate and optimize a hybrid rainwater-harvesting and aqueduct-backup irrigation system tailored for shallow clay-loam soils under stochastic bimodal precipitation.

## 📌 Core Features

* **Strict Mathematical Isomorphism:** The code acts as a direct digital twin to the mathematical formulation presented in the manuscript, ensuring complete phenomenological and dimensional consistency.
* **Hybrid Automaton Implementation:** The hysteresis loop (Layer 2) is formally implemented as a hybrid automaton with a discrete memory state $m(t)$ and a precise non-smooth jump map.
* **Exact LCP Complementarity:** Rainwater tank dynamics ($V_r$) are solved using exact Linear Complementarity Problem (LCP) conditions, guaranteeing strict mass balance closure during overflow and rigorous cavitation protection ($V_{\min}$).
* **Stochastic Bimodal Forcing:** Includes a custom compound Poisson process generator calibrated to local IDEAM climatology ($1213 \text{ mm/year}$).
* **Thermodynamic WDEL Coupling:** Integrates an empirical, temperature-coupled transfer function for Wind Drift and Evaporation Losses (WDEL), dynamically limiting sprinkler efficiency $\eta(t)$.

## 🏗️ Control Architecture

The system operates under a **Rain-Priority State-Feedback Heuristic**, structured in three layers:
1. **Supervisory Control:** Hard constraints to prevent pump cavitation.
2. **Regulatory Control:** A discrete memory relay enforcing agronomic bounds (Field Capacity to 50% Management Allowed Depletion) alongside a strict nocturnal watering window.
3. **Resource Allocation:** A deterministic state-feedback law prioritizing harvested rainwater over the municipal aqueduct to minimize hydro-economic Operational Expenditure (OPEX).

## 🚀 Usage

### Prerequisites
* **MATLAB/MATLAB Online** (Base installation is sufficient. No additional toolboxes like Simulink or Control System Toolbox are strictly required as the ODEs are solved via custom explicit integration).

### Running the Simulation
Just download/copy the code and run it in your prefered Matlab enviroment, *bear in mind in the code there's a specific seed setted for reproducibility, you can change this later to see how it affects the model's response to the simulated climatic behavior.*


### 📊 Output and Visualization

The script integrates the system over a Tsim​=365×1440 minutes horizon (evaluated at Δt=1 min) and outputs a comprehensive 6-panel hydro-economic figure (fig1):
*    Panel A (Temperature): Diurnal thermal forcing driving the system's efficiency.
*    Panel B (Rainfall): Simulated bimodal precipitation profile.
*    Panel C (Storage Regulation): Root-zone water storage (H(t)). Compares the closed-loop hybrid controller against a fixed-schedule open-loop baseline, highlighting the mitigation of deep percolation and waterlogging.
*    Panel D (Tank Dynamics): Rainwater harvesting tank volume (Vr​(t)) bounded by cavitation and overflow thresholds.
*    Panel E (Monthly Consumption): Stacked bar chart of volumetric water origin (Rainwater vs. Aqueduct).
*    Panel F (Cumulative OPEX): Financial divergence demonstrating the aqueduct-water expenditure savings (e.g., 67.4%).

### 📖 Academic Context & Citation

This code was developed to support the findings submitted to the MAPI4 proceedings. The model utilizes phenomenological approximations suitable for low-computational-burden embedded systems.

If you utilize this code for your research, please consider citing the associated paper once published.

Developed by Sergio Andrés Castro Acuña - Universidad Nacional de Colombia.
