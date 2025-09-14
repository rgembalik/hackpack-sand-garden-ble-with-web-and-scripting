#pragma once
// Shared definition of Positions struct used across motion, scripting, and patterns.
// Keep this minimal to avoid pulling large dependencies into small modules.
struct Positions {
  int radial;  // motor steps (radial axis)
  int angular; // motor steps (angular axis)
};
