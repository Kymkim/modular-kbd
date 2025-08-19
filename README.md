# MSKBD - Modularly Simple KeyBoarD

> [!WARNING]  
> This is under development. If you want the latest developments check out the `dev` branch.

A customizable modular keyboard system where each key cluster is its own module, connected through a grid-like commnication. The goal is to create a flexible keyboard that can scale, reconfigure layouts, and support hot-swapping modules.

## Features
- 🧩 **Modular design** — attach modules in any direction
- 🎛️ **Configurable firmware** — master + module code, easily updated
- ⚡ **Hotplug detection** — modules can be discovered dynamically

## Repo Structure
```
├─firmware/ - Embedded code for the core system
├─hardware/ - Schematics and PCB layouts
├─software/ - Host tools (keymap editor, etc.) (not yet implemented - soon hopefully!)
├─docs/ - Design notes, communication protocol, build guides
└─assets/ - Renders, diagrams, and media
```

## Branch Structure
```
main              # Stable, production-ready code (only tested & reviewed changes go here)
│
├─ dev            # Integration branch for features; "staging" area before merging to main
│  ├─ software-xxxxxx     # New features, experiments, improvements, or PCB uploads    
|  ├─ firmware-xxxxxx     # Please indicate in the branch the wether if its software
│  └─ hardware-xxxxxx     # firmware, or hardware
│
├─ hotfix/        # Quick fixes for urgent bugs in main
│   └─ hotfix/usb-detection
│
└─ docs/          # Documentation updates
    └─ docs/protocol-update
```

## Contributors
<a href="https://github.com/Kymkim/modular-kbd/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Kymkim/modular-kbd" />
</a>
