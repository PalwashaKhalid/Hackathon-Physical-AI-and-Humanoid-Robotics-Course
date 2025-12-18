# Claude Code Subagents and Agent Skills System
## Reusable Intelligence for Physical AI & Humanoid Robotics Book

### Overview
The Claude Code Subagents and Agent Skills system provides reusable intelligence components for the Physical AI & Humanoid Robotics book project. This system automates common tasks, ensures quality, and maintains consistency across documentation, code, and AI components.

### Architecture

#### Subagent Orchestration
The system is built around a central orchestrator that manages:
- Skill registration and execution
- Event-driven triggering
- Configuration management
- Logging and monitoring

#### Skill Categories
Skills are organized into categories based on functionality:

1. **Documentation Skills** - Handle document validation, cross-referencing, and formatting
2. **RAG Skills** - Manage vector indexing and content retrieval
3. **Quality Skills** - Ensure technical accuracy and consistency
4. **Frontend Skills** - Validate React components and UI integration
5. **API Skills** - Validate and optimize API endpoints
6. **Build Skills** - Manage dependencies and build processes
7. **AI Skills** - Optimize model performance and manage embeddings

### Installation and Setup

#### Prerequisites
- Python 3.8+
- Node.js (for Docusaurus integration)
- The existing book project dependencies

#### Setup Steps
1. Navigate to the project root:
   ```bash
   cd path/to/your/book/project
   ```

2. The subagents system is already integrated in `.claude/subagents/`

3. Ensure the required dependencies are available (they should already be in your project):
   - FastAPI (for API endpoints)
   - Python standard library components

### Usage

#### Command Line Interface
The system provides a command-line interface for direct interaction:

```bash
# List all available skills
python -m .claude.subagents.cli list

# Execute a specific skill
python -m .claude.subagents.cli skill doc-validator --params '{"files": ["docs/chapter1-fundamentals.md"]}'

# Trigger an event
python -m .claude.subagents.cli event file-change --data '{"file_path": "docs/new-chapter.md", "change_type": "create"}'

# Validate documentation files
python -m .claude.subagents.cli validate docs/chapter1-fundamentals.md docs/chapter2-ros2.md

# Check system status
python -m .claude.subagents.cli status
```

#### API Interface
The system also provides a REST API for programmatic access:

```bash
# Start the API server
python -m .claude.subagents.api

# List available skills
curl http://localhost:8081/api/skills

# Execute a skill
curl -X POST http://localhost:8081/api/skill/trigger \
  -H "Content-Type: application/json" \
  -d '{"skill_id": "doc-validator", "parameters": {"files": ["docs/chapter1-fundamentals.md"]}}'
```

### Available Skills

#### Documentation Skills
- **doc-validator**: Validates documentation structure, content, and formatting consistency
- **auto-linker**: Automatically creates cross-references between related content
- **format-checker**: Ensures consistent formatting across all documentation files
- **content-updater**: Updates content based on source code changes

#### RAG Skills
- **vector-indexer**: Automatically indexes new content when documentation is added
- **similarity-checker**: Identifies similar content across chapters to suggest cross-references
- **embedding-updater**: Refreshes embeddings when content changes
- **search-optimizer**: Optimizes search parameters based on query patterns

#### Quality Skills
- **tech-fact-checker**: Verifies technical accuracy against known sources
- **terminology-consistency**: Ensures consistent use of technical terms across chapters
- **concept-linker**: Identifies related concepts that should reference each other
- **error-detector**: Finds technical errors or inconsistencies in explanations

### Configuration

The system is configured through `.claude/subagents/config.json`:

```json
{
  "subagents_system": {
    "version": "1.0.0",
    "enabled": true,
    "debug_mode": false
  },
  "event_system": {
    "file_watchers": {
      "docs_directory": {
        "path": "./docs",
        "extensions": [".md"],
        "events": ["create", "update", "delete"]
      },
      "api_directory": {
        "path": "./rag_chatbot/api",
        "extensions": [".py"],
        "events": ["create", "update", "delete"]
      }
    },
    "git_hooks": {
      "pre_commit": ["doc-validator", "api-validator"],
      "post_commit": ["vector-indexer"]
    },
    "scheduled_tasks": {
      "daily": ["tech-fact-checker", "consistency-checker"],
      "weekly": ["model-updater", "dependency-manager"],
      "monthly": ["performance-audit"]
    }
  },
  "skill_registry": {
    "base_path": "./.claude/subagents/skills",
    "categories": ["documentation", "rag", "quality", "frontend", "api", "build", "ai"]
  },
  "execution": {
    "max_concurrent_tasks": 5,
    "timeout_seconds": 300,
    "retry_attempts": 3
  },
  "logging": {
    "level": "info",
    "output_file": "./.claude/subagents/logs/subagents.log",
    "retention_days": 30
  },
  "api": {
    "enabled": true,
    "port": 8081,
    "endpoints": {
      "trigger_skill": "/api/skill/trigger",
      "get_status": "/api/status",
      "list_skills": "/api/skills"
    }
  }
}
```

### Integration with Existing Systems

#### Docusaurus Integration
The subagents system works alongside your Docusaurus documentation site:
- Automatically validates documentation when changes are made
- Updates the RAG system when new content is added
- Maintains cross-references between related topics

#### RAG System Integration
The subagents system enhances your existing RAG functionality:
- Automatically indexes new content
- Optimizes search performance
- Maintains content quality

### Event-Driven Architecture

The system uses an event-driven approach to automate tasks:

#### File System Events
- Watches for changes in documentation files
- Triggers validation when files are created or modified
- Updates the RAG system when content changes

#### Git Integration
- Runs validation checks on pre-commit hooks
- Updates indexes on post-commit events
- Maintains consistency across versions

#### Scheduled Tasks
- Daily quality checks on content
- Weekly dependency updates
- Monthly performance audits

### Extending the System

#### Adding New Skills
To add a new skill:

1. Create a JSON configuration file in the appropriate category directory:
   ```json
   {
     "skillId": "my-new-skill",
     "name": "My New Skill",
     "description": "Description of what the skill does",
     "category": "documentation",
     "triggers": ["file-change", "manual"],
     "parameters": {
       "required": ["param1"],
       "optional": {
         "param2": "default_value"
       }
     },
     "dependencies": [],
     "output": {
       "format": "json",
       "schema": {}
     },
     "implementation": {
       "type": "claude-task",
       "task_type": "custom",
       "script": "my_skill_script.py"
     }
   }
   ```

2. Create the implementation script in the same directory

3. The skill will be automatically registered when the system starts

#### Custom Configuration
You can customize the behavior by modifying the configuration file:
- Adjust scheduling intervals
- Enable/disable specific skills
- Configure logging and monitoring

### Best Practices

1. **Consistency**: Use the subagents system to maintain consistency across all documentation
2. **Automation**: Leverage event-driven triggers to automate repetitive tasks
3. **Validation**: Run validation skills regularly to catch issues early
4. **Monitoring**: Check logs regularly to ensure the system is functioning properly
5. **Documentation**: Keep skill documentation up to date when adding new functionality

### Troubleshooting

#### Common Issues
- **Skill not found**: Ensure the skill is properly registered in the correct category directory
- **Configuration errors**: Validate JSON syntax in configuration files
- **Permission issues**: Ensure the system has read/write access to necessary directories

#### Logging
Check the log file at `.claude/subagents/logs/subagents.log` for detailed information about system operations and errors.

### Benefits

The Claude Code Subagents and Agent Skills system provides:

1. **Consistency**: Ensures consistent formatting, terminology, and quality across all book chapters
2. **Efficiency**: Automates repetitive tasks like content indexing and validation
3. **Quality**: Catches errors and inconsistencies automatically
4. **Scalability**: Handles growth in content without proportional increase in manual work
5. **Integration**: Maintains tight coupling between documentation and the RAG system
6. **Maintainability**: Provides reusable components that can be easily updated and extended

This system transforms your Physical AI & Humanoid Robotics book project into an intelligent, self-managing documentation system that maintains high quality while reducing manual effort.