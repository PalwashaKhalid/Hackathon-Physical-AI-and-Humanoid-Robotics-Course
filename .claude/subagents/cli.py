#!/usr/bin/env python3
"""
Command Line Interface for Claude Code Subagents System
Provides easy access to subagent functionality from the command line
"""

import argparse
import asyncio
import json
import sys
from pathlib import Path

# Add the subagents directory to the path to import the orchestrator
import os
subagents_dir = Path(__file__).parent
sys.path.append(str(subagents_dir))
from subagent_orchestrator import SubagentOrchestrator


async def main():
    parser = argparse.ArgumentParser(description="Claude Code Subagents CLI for Physical AI & Humanoid Robotics Book")
    parser.add_argument("--config", default=".claude/subagents/config.json", help="Path to config file")

    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Skill execution command
    skill_parser = subparsers.add_parser("skill", help="Execute a specific skill")
    skill_parser.add_argument("skill_id", help="ID of the skill to execute")
    skill_parser.add_argument("--params", type=str, help="JSON string of parameters")
    skill_parser.add_argument("--params-file", type=str, help="File containing parameters as JSON")

    # Event triggering command
    event_parser = subparsers.add_parser("event", help="Trigger skills based on an event")
    event_parser.add_argument("event_type", help="Type of event to trigger")
    event_parser.add_argument("--data", type=str, help="JSON string of event data")
    event_parser.add_argument("--data-file", type=str, help="File containing event data as JSON")

    # List skills command
    list_parser = subparsers.add_parser("list", help="List all available skills")

    # Validate documentation command
    validate_parser = subparsers.add_parser("validate", help="Validate documentation files")
    validate_parser.add_argument("files", nargs="*", help="Files to validate (if none, validates all docs)")

    # Status command
    status_parser = subparsers.add_parser("status", help="Get system status")

    args = parser.parse_args()

    orchestrator = SubagentOrchestrator(config_path=args.config)

    if args.command == "skill":
        # Parse parameters
        params = {}
        if args.params:
            try:
                params = json.loads(args.params)
            except json.JSONDecodeError:
                print("Error: Invalid JSON in parameters", file=sys.stderr)
                sys.exit(1)
        elif args.params_file:
            try:
                with open(args.params_file, 'r') as f:
                    params = json.load(f)
            except json.JSONDecodeError:
                print("Error: Invalid JSON in parameters file", file=sys.stderr)
                sys.exit(1)
            except FileNotFoundError:
                print(f"Error: Parameters file not found: {args.params_file}", file=sys.stderr)
                sys.exit(1)

        result = await orchestrator.execute_skill(args.skill_id, params)
        print(json.dumps(result, indent=2))

    elif args.command == "event":
        # Parse event data
        event_data = {}
        if args.data:
            try:
                event_data = json.loads(args.data)
            except json.JSONDecodeError:
                print("Error: Invalid JSON in event data", file=sys.stderr)
                sys.exit(1)
        elif args.data_file:
            try:
                with open(args.data_file, 'r') as f:
                    event_data = json.load(f)
            except json.JSONDecodeError:
                print("Error: Invalid JSON in event data file", file=sys.stderr)
                sys.exit(1)
            except FileNotFoundError:
                print(f"Error: Event data file not found: {args.data_file}", file=sys.stderr)
                sys.exit(1)

        results = await orchestrator.trigger_event(args.event_type, event_data)
        print(json.dumps(results, indent=2))

    elif args.command == "list":
        skills = list(orchestrator.skill_registry.keys())
        categories = {}
        for skill_id, skill_info in orchestrator.skill_registry.items():
            category = skill_info['config'].get('category', 'unknown')
            if category not in categories:
                categories[category] = []
            categories[category].append(skill_id)

        print("Available Skills:")
        for category, skill_list in categories.items():
            print(f"\n  {category.upper()}:")
            for skill in skill_list:
                skill_config = orchestrator.skill_registry[skill]['config']
                print(f"    - {skill}: {skill_config.get('description', 'No description')}")

    elif args.command == "validate":
        # Special handling for documentation validation
        if not args.files:
            # Validate all documentation files
            docs_dir = Path("docs")
            if docs_dir.exists():
                args.files = [str(f) for f in docs_dir.glob("*.md")]
            else:
                print("No documentation files found in 'docs' directory", file=sys.stderr)
                sys.exit(1)

        # Import the documentation validator directly
        import sys
        import os
        docs_validator_path = os.path.join(subagents_dir, "skills", "documentation")
        sys.path.append(docs_validator_path)
        from validate_docs import DocumentationValidator

        validator = DocumentationValidator()
        result = validator.validate_multiple_files(args.files)
        print(json.dumps(result, indent=2))

    elif args.command == "status":
        print(f"Subagents System Status:")
        print(f"  Config loaded: {bool(orchestrator.config)}")
        print(f"  Total skills: {len(orchestrator.skill_registry)}")
        print(f"  Categories: {len(set(skill_info['config'].get('category', 'unknown') for skill_info in orchestrator.skill_registry.values()))}")
        print(f"  Enabled: {orchestrator.config.get('subagents_system', {}).get('enabled', False)}")

    else:
        parser.print_help()


if __name__ == "__main__":
    asyncio.run(main())