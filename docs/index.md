---
title: Kinova Middleware Manual
hide:
  - navigation
  - toc
---

<div class="hero" markdown>
<div class="hero__text" markdown>

# Type a sentence.<br>The arm does it, or says no.

A language model writes the plan. This middleware checks it, then moves a Kinova Gen3 Lite arm with ROS 2 and MoveIt.

[Quickstart](user-guide/quickstart.md){ .md-button .md-button--primary }
[Install](installation/middleware-and-proxy.md){ .md-button }
[How it works](overview.md){ .md-button }

</div>
<img class="hero__arm" src="assets/arm.png" alt="Kinova Gen3 Lite arm reaching down to pick up an object">
</div>

## Start here

<div class="grid cards paths" markdown>

-   **User guide**

    Run the system and give it commands.

    [Quickstart](user-guide/quickstart.md) · [Troubleshooting](user-guide/troubleshooting.md) · [Demos](user-guide/demos.md)

-   **Installation**

    Set up ROS 2, the Kinova drivers, the middleware and the proxy.

    [Middleware and proxy](installation/middleware-and-proxy.md) · [Non-native Ubuntu](installation/non-native-ubuntu.md) · [Configuration](installation/configuration.md)

-   **Technical**

    How it works inside, and the code reference.

    [Architecture](technical/architecture.md) · [Safety and contracts](technical/safety-and-contracts.md) · [API reference](technical/api/nodes.md)

</div>

## From sentence to motion

![You type a sentence, the model writes a recipe, checks run, ROS 2 runs each step, MoveIt plans and the arm moves](assets/flow.svg){ .flow }

## The ten actions

<div class="verbs" markdown>
`home` `move` `relative move` `gripper` `pick up` `drop off` `pour` `push` `thrust` `throw`
</div>

Anything else the model suggests is rejected before the arm moves.

<p class="credit">Arm render from Kinova's open Kortex model (Apache-2.0).</p>
