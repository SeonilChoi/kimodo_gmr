# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# ruff: noqa: I001
import argparse

from kimodo.model.registry import resolve_model_name

from .app import Demo


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the kimodo demo UI.")
    parser.add_argument(
        "--model",
        type=str,
        default="Kimodo-SMPLX-RP-v1",
        help="Default model to load (default: Kimodo-SMPLX-RP-v1; e.g. Kimodo-SOMA-RP-v1, kimodo-soma-rp, or SOMA).",
    )
    args = parser.parse_args()

    resolved = resolve_model_name(args.model, "Kimodo")
    demo = Demo(default_model_name=resolved)
    demo.run()


if __name__ == "__main__":
    main()
