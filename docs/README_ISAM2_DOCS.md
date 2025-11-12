# ISAM2 Documentation Index

This directory contains comprehensive documentation about ISAM2 (Incremental Smoothing and Mapping 2) and its role in your Kimera-VIO integration.

## 📚 Documents Overview

### 1. **ISAM2_CHEAT_SHEET.md** ⚡ START HERE
Quick reference for immediate use. Contains:
- One-minute summary
- Essential code patterns
- Common operations
- Troubleshooting guide
- Configuration examples

**Best for**: Quick lookups, copy-paste code snippets, debugging issues

---

### 2. **ISAM2_TUTORIAL.md** 📖 COMPREHENSIVE GUIDE
In-depth explanation of ISAM2 theory and practice. Contains:
- What ISAM2 is and why it matters
- Core concepts (factor graphs, Bayes Tree, linearization)
- How ISAM2 works internally
- Your codebase integration details
- Kimera integration context
- Performance considerations
- Configuration parameters explained

**Best for**: Understanding the theory, learning how it works, troubleshooting complex issues

---

### 3. **ISAM2_VISUAL_REFERENCE.md** 🎨 VISUAL GUIDE
Diagrams and visual examples. Contains:
- Factor graph evolution over time
- Marginalization visualization
- Relinearization process
- Memory layout comparisons
- Complexity comparisons
- Sparsity patterns
- Sequence diagrams

**Best for**: Visual learners, understanding the process flow, teaching others

---

### 4. **KIMERA_INTEGRATION_QUICK_REFERENCE.md** 🔗 INTEGRATION GUIDE
(Updated with ISAM2 section)
Kimera-specific integration details. Contains:
- How ISAM2 fits into Kimera workflow
- What gets optimized
- Parameter mapping
- Performance characteristics
- Debugging tips

**Best for**: Understanding the complete Kimera integration, seeing ISAM2 in context

---

## 🎯 Learning Path

### Path 1: "I Need to Use It Now"
1. Read **ISAM2_CHEAT_SHEET.md** (10 minutes)
2. Copy configuration template
3. Start coding
4. Refer back for troubleshooting

### Path 2: "I Want to Understand It"
1. Read **ISAM2_TUTORIAL.md** sections 1-3 (30 minutes)
2. Look at **ISAM2_VISUAL_REFERENCE.md** examples (15 minutes)
3. Read **ISAM2_TUTORIAL.md** sections 4-7 (30 minutes)
4. Review **KIMERA_INTEGRATION_QUICK_REFERENCE.md** ISAM2 section (10 minutes)

### Path 3: "Deep Dive"
1. Complete Path 2 above
2. Read original ISAM2 paper (linked in tutorial)
3. Explore GTSAM source code: `gtsam/nonlinear/ISAM2.cpp`
4. Study your codebase: `IncrementalFixedLagSmoother.cpp`
5. Experiment with parameters and observe results

---

## 🔍 Quick Navigation

### Finding Answers to Common Questions

**"How do I configure ISAM2?"**
→ ISAM2_CHEAT_SHEET.md → Configuration File Template
→ ISAM2_TUTORIAL.md → Configuration Parameters section

**"Why is optimization slow?"**
→ ISAM2_CHEAT_SHEET.md → Troubleshooting → Problem: Too Slow
→ ISAM2_TUTORIAL.md → Performance Considerations

**"What's the difference between ISAM2 and batch optimization?"**
→ ISAM2_TUTORIAL.md → Core Concepts
→ ISAM2_VISUAL_REFERENCE.md → Optimization Complexity Comparison

**"How does this work with Kimera?"**
→ KIMERA_INTEGRATION_QUICK_REFERENCE.md → ISAM2 Optimization Details
→ ISAM2_VISUAL_REFERENCE.md → Kimera Integration Data Flow

**"What do these parameters mean?"**
→ ISAM2_CHEAT_SHEET.md → Key Parameters
→ ISAM2_TUTORIAL.md → Configuration Parameters

**"How does marginalization work?"**
→ ISAM2_VISUAL_REFERENCE.md → Time t=5: Fixed-Lag Marginalization
→ ISAM2_TUTORIAL.md → How ISAM2 Works

**"What's a Bayes Tree?"**
→ ISAM2_TUTORIAL.md → Core Concepts → Bayes Tree
→ ISAM2_VISUAL_REFERENCE.md → Factor Graph Evolution

**"My optimization is diverging!"**
→ ISAM2_CHEAT_SHEET.md → Troubleshooting → Problem: Optimization Diverges
→ ISAM2_TUTORIAL.md → Common Issues and Solutions

---

## 💡 Key Takeaways

### The Big Picture
ISAM2 is your incremental optimizer that:
1. **Receives**: Factor graphs (states + measurements)
2. **Maintains**: Bayes Tree for efficient updates
3. **Outputs**: Optimized state estimates
4. **Manages**: Automatic marginalization and relinearization

### Why It Matters for Kimera
- **Real-time**: Fast enough for online operation (10-50ms per update)
- **Incremental**: Handles streaming data from Kimera
- **Fixed-lag**: Constant memory/time with marginalization
- **Accurate**: Maintains global consistency while being fast

### The Critical Insight
```
Batch Optimization:  O(n³) time - infeasible for real-time
                     ↓
ISAM2 Incremental:   O(m·k²) time - 100-1000x faster!
                     ↓
Fixed-Lag Smoother:  O(1) time - constant with marginalization!
```

This is why your system can run in real-time!

---

## 📊 Document Stats

| Document | Pages | Words | Time to Read | Difficulty |
|----------|-------|-------|--------------|------------|
| Cheat Sheet | 8 | 2,500 | 10 min | Easy |
| Tutorial | 35 | 10,000 | 60 min | Medium |
| Visual Reference | 15 | 4,000 | 30 min | Easy |
| Kimera Integration | 5 | 1,500 | 10 min | Medium |

**Total Learning Time**: ~2 hours for complete understanding

---

## 🛠️ Related Code Files

### Core ISAM2 Implementation
```
gtsam/gtsam/nonlinear/
├── ISAM2.h                    # Main ISAM2 class
├── ISAM2.cpp                  # Implementation
├── ISAM2Params.h              # Configuration parameters
├── ISAM2Result.h              # Result structure
└── ISAM2-impl.h               # Internal algorithms
```

### Your Fixed-Lag Smoother
```
online_fgo_core/
├── include/online_fgo_core/solver/
│   ├── IncrementalFixedLagSmoother.h     # Wrapper with marginalization
│   └── FixedLagSmoother.h                # Base class
└── src/solver/
    └── IncrementalFixedLagSmoother.cpp   # Implementation
```

### Graph Integration
```
online_fgo_core/
├── include/online_fgo_core/graph/
│   ├── GraphBase.h                       # Base graph with ISAM2 setup
│   └── GraphTimeCentric[Kimera].h        # Your graph class
└── src/graph/
    └── GraphBase.cpp                     # ISAM2 configuration
```

### Kimera Adapter
```
Kimera-VIO/include/kimera-vio/integration/
└── GraphTimeCentricBackendAdapter.h      # Calls your ISAM2 backend
```

---

## 🔗 External Resources

### Official GTSAM
- **Website**: https://gtsam.org/
- **Documentation**: https://gtsam.org/doxygen/
- **Tutorials**: https://gtsam.org/tutorials/intro.html
- **GitHub**: https://github.com/borglab/gtsam

### Academic Papers
- **ISAM2 Paper**: Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree" (IJRR 2012)
- **Factor Graphs**: Dellaert & Kaess, "Factor Graphs for Robot Perception" (FnT Robotics 2017)
- **Fixed-Lag**: Williams et al., "Incremental Fixed-Lag Smoothing" (ICRA 2012)

### Video Lectures
- Frank Dellaert's Factor Graph lectures (YouTube)
- GTSAM tutorials from Georgia Tech

---

## 🤝 Contributing

Found an error or want to add clarification?
1. Edit the relevant markdown file
2. Keep the conversational, tutorial style
3. Add visual examples where helpful
4. Update this index if adding new sections

---

## ❓ Still Have Questions?

### Debug Checklist
1. ✅ Read the cheat sheet troubleshooting section
2. ✅ Check visual reference for diagram of your scenario
3. ✅ Search tutorial for keyword
4. ✅ Look at code examples in your codebase
5. ✅ Enable `enableDetailedResults` and check ISAM2Result
6. ✅ Print factor graph and values to inspect
7. ❓ Still stuck? Check GTSAM GitHub issues or forums

### Common Debugging Commands
```cpp
// Enable verbose logging
params.enableDetailedResults = true;

// Print everything
isam.print("ISAM2 state:");
factors.print("Factors:");
values.print("Values:");

// Check specific aspects
LOG(INFO) << "Error: " << result.errorAfter.value();
LOG(INFO) << "Variables: " << isam.size();
LOG(INFO) << "Relinearized: " << result.variablesRelinearized;
```

---

## 📝 Quick Command Reference

```cpp
// Include
#include <gtsam/nonlinear/ISAM2.h>
#include "online_fgo_core/solver/IncrementalFixedLagSmoother.h"

// Setup
ISAM2Params params;
params.relinearizeThreshold = 0.05;
IncrementalFixedLagSmoother smoother(3.0, params);

// Update
smoother.update(factors, values, timestamps);

// Get results
Values optimized = smoother.calculateEstimate();
Pose3 pose = optimized.at<Pose3>(X(i));
```

---

**Last Updated**: November 2025
**Maintainer**: Documentation for ETHZ V4RL online_fgo_core project
**Version**: 1.0
