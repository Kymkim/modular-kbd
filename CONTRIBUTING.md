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
