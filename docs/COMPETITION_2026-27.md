# 2026–27 Competition Constraints — GNC-Relevant

**Status: CURRENT — 2026–27 competition requirements**

This file contains the rules that materially affect autonomy/GNC. The official NASA guidebook remains authoritative.

## Navigation Restrictions
- **GPS**, GPS-enabled IMU data, and **compasses may not contribute to internal robot calculations**.
- Passive fiducials / permitted navigation aids may be placed only in the designated arena-frame area under the guidebook rules.
- Prior knowledge of arena walls/column geometry may not be used as the basis for autonomous mapping/navigation. Newly detected features may contribute only within the allowed interpretation.
- Touch-sensor/wall-following approaches are not allowed for autonomous navigation.

## Travel Autonomy
The obstacle field is intended to require:
- obstacle detection
- mapping
- navigation/path planning
- a nontrivial slalom-style route

A simple point-and-traverse architecture is not sufficient.

Mission Control must display **real-time obstacle detection, associated obstacle mapping, and the resulting path plan**. No real-time human interaction may control or influence that visualization/autonomy system during the attempt.

## Autonomy Scoring
- Up to **1,400 points** are available for full autonomy.
- Full autonomy requires hands-free operation after localization and at least two complete cycles.
- A full cycle is: **excavation -> loaded travel -> dump -> empty travel -> repeat**.
- Autonomy scoring is multiplied by a berm-volume factor.
- **25,000 cm^3** of berm volume gives a 1.0 multiplier; below that, the multiplier is proportional to berm volume; zero berm volume yields zero autonomy points.

## Design Consequence
2025–26 planners, filters, sensors, and control logic must be revalidated against these rules before being marked current.
