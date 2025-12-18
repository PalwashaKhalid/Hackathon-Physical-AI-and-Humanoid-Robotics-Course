#!/usr/bin/env python3
"""
Documentation Validator Skill Implementation
Validates documentation structure, content, and formatting consistency
"""

import re
import json
from pathlib import Path
from typing import List, Dict, Any, Tuple
import markdown
from markdown.treeprocessors import Treeprocessor
from markdown.extensions import Extension


class DocumentationValidator:
    def __init__(self):
        self.issues = []
        self.frontmatter_pattern = re.compile(r'^---\s*\n(.*?)\n---\s*\n', re.DOTALL)
        self.header_pattern = re.compile(r'^(#{1,6})\s+(.+)$', re.MULTILINE)
        self.code_block_pattern = re.compile(r'```(\w*)\n(.*?)```', re.DOTALL)

    def validate_file(self, file_path: str) -> Dict[str, Any]:
        """Validate a single documentation file."""
        self.issues = []
        file_path_obj = Path(file_path)

        if not file_path_obj.exists():
            return {
                'valid': False,
                'issues': [{'file': file_path, 'issue': 'File does not exist', 'severity': 'error', 'line': 0}]
            }

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except Exception as e:
            return {
                'valid': False,
                'issues': [{'file': file_path, 'issue': f'Cannot read file: {str(e)}', 'severity': 'error', 'line': 0}]
            }

        # Validate frontmatter
        self._validate_frontmatter(content, file_path)

        # Validate headers
        self._validate_headers(content, file_path)

        # Validate code blocks
        self._validate_code_blocks(content, file_path)

        # Validate links
        self._validate_links(content, file_path)

        # Validate overall structure
        self._validate_structure(content, file_path)

        return {
            'valid': len(self.issues) == 0,
            'issues': self.issues,
            'summary': {
                'total_issues': len(self.issues),
                'errors': len([i for i in self.issues if i['severity'] == 'error']),
                'warnings': len([i for i in self.issues if i['severity'] == 'warning']),
                'file': file_path
            }
        }

    def validate_multiple_files(self, file_paths: List[str]) -> Dict[str, Any]:
        """Validate multiple documentation files."""
        all_results = []
        all_issues = []
        valid_count = 0

        for file_path in file_paths:
            result = self.validate_file(file_path)
            all_results.append(result)
            all_issues.extend(result['issues'])
            if result['valid']:
                valid_count += 1

        return {
            'valid': len(all_issues) == 0,
            'files_validated': len(file_paths),
            'files_valid': valid_count,
            'files_invalid': len(file_paths) - valid_count,
            'issues': all_issues,
            'summary': {
                'total_issues': len(all_issues),
                'errors': len([i for i in all_issues if i['severity'] == 'error']),
                'warnings': len([i for i in all_issues if i['severity'] == 'warning'])
            }
        }

    def _validate_frontmatter(self, content: str, file_path: str):
        """Validate YAML frontmatter."""
        match = self.frontmatter_pattern.search(content)
        if not match:
            self.issues.append({
                'file': file_path,
                'issue': 'Missing YAML frontmatter',
                'severity': 'warning',
                'line': 1
            })
        else:
            frontmatter = match.group(1)
            try:
                # Basic YAML validation (in a real implementation, use pyyaml)
                if 'sidebar_position:' not in frontmatter:
                    self.issues.append({
                        'file': file_path,
                        'issue': 'Missing sidebar_position in frontmatter',
                        'severity': 'warning',
                        'line': 1
                    })
            except:
                self.issues.append({
                    'file': file_path,
                    'issue': 'Invalid YAML in frontmatter',
                    'severity': 'error',
                    'line': 1
                })

    def _validate_headers(self, content: str, file_path: str):
        """Validate markdown headers."""
        lines = content.split('\n')
        header_levels = []

        for i, line in enumerate(lines, 1):
            header_match = self.header_pattern.match(line)
            if header_match:
                level = len(header_match.group(1))
                header_levels.append((level, header_match.group(2), i))

                # Check if header level jumps too much (e.g., from h1 to h4)
                if len(header_levels) > 1:
                    prev_level = header_levels[-2][0]
                    if level > prev_level + 1:
                        self.issues.append({
                            'file': file_path,
                            'issue': f'Header level jumps from {prev_level} to {level}',
                            'severity': 'warning',
                            'line': i
                        })

    def _validate_code_blocks(self, content: str, file_path: str):
        """Validate code blocks."""
        code_blocks = self.code_block_pattern.findall(content)

        for i, (lang, code) in enumerate(code_blocks):
            if not lang:
                self.issues.append({
                    'file': file_path,
                    'issue': f'Code block at position {i+1} missing language specification',
                    'severity': 'warning',
                    'line': self._get_line_number(content, code)
                })

    def _validate_links(self, content: str, file_path: str):
        """Validate markdown links."""
        # Check for broken relative links
        link_pattern = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')
        links = link_pattern.findall(content)

        for link_text, link_url in links:
            if link_url.startswith('./') or link_url.startswith('../'):
                # This is a relative link, check if target exists
                link_path = Path(file_path).parent / link_url
                if not link_path.exists():
                    self.issues.append({
                        'file': file_path,
                        'issue': f'Broken relative link: {link_url}',
                        'severity': 'error',
                        'line': self._get_line_number(content, link_url)
                    })

    def _validate_structure(self, content: str, file_path: str):
        """Validate overall document structure."""
        # Check for minimum content length
        if len(content.strip()) < 50:
            self.issues.append({
                'file': file_path,
                'issue': 'Document appears to have very little content',
                'severity': 'warning',
                'line': 1
            })

        # Check for proper sentence structure
        sentences = re.split(r'[.!?]+', content)
        long_sentences = [s for s in sentences if len(s) > 200]
        if long_sentences:
            self.issues.append({
                'file': file_path,
                'issue': f'Document contains {len(long_sentences)} very long sentences (>200 chars)',
                'severity': 'warning',
                'line': 1
            })

    def _get_line_number(self, content: str, target: str) -> int:
        """Get the line number of a target string in content."""
        lines = content.split('\n')
        for i, line in enumerate(lines, 1):
            if target in line:
                return i
        return 1


def main():
    """Main function to run the documentation validator."""
    import sys
    import json

    if len(sys.argv) < 2:
        print("Usage: python validate_docs.py <file1> [file2] ...")
        sys.exit(1)

    files_to_validate = sys.argv[1:]
    validator = DocumentationValidator()

    if len(files_to_validate) == 1:
        result = validator.validate_file(files_to_validate[0])
    else:
        result = validator.validate_multiple_files(files_to_validate)

    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()