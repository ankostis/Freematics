# JRCMATICS Copilot Instructions

## SUMMARY.md - Git Commit Message Draft

Maintain in repo root as working doc for commits:
- Clear when describing new changes (ask if unsure)
- Document staged/uncommitted changes (specify which)
- Use `git diff --staged` to check index (not `get_changed_files` API)
- Lines <90 chars, concise, suitable for git commit

Header line (1st line):
- Syntax: `type(scope) phrase` or `type: phrase` (skip `:` if scope present)
- Types: feat/enh/refact/drop/chore/doc/style, lowercase scope
- Scope can be `tests`/`TCs` (no `test` type exists)
- <50ish chars, end with `>` if body follows
- Separate distinct changes with `;`
- Loosely follow "conventional commits"

Body (after header):
- Simple/direct tone: present tense (changes), past tense (bugs/old behavior)
- Big picture first, be succinct, use "etc" freely
- What changed and why (skip file listings, reviewer sees diffs)
- NO markdown headers (#), confuses vim rebase-interactive
- Backticks for code, italics for concepts, bold for emphasis
- Bullet points for actions/todos, paragraphs for explanations
- TODOs/performance/test changes at bottom if present
- Include "Why" only when non-obvious (workarounds, arch decisions, timing)
