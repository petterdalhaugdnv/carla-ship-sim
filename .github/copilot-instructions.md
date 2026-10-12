## Git workflow

- Before editing, inspect the current branch, git status, and applicable
  repository instructions.
- Preserve existing user changes. Never discard or commit unrelated work.
- Work on a task branch. If multiple agents are editing concurrently,
  each must use a separate Git worktree and branch.
- Keep changes within the assigned task. Avoid unrelated refactoring.
- Make focused commits at logical checkpoints, with descriptive messages.
- Stage specific files rather than using `git add .`.
- Review the staged diff before committing.
- Commit relevant documentation with the corresponding implementation.
- Run appropriate checks and report their actual results.
- Never force-push, reset destructively, merge into marine-main, or update
  upstream dependencies unless explicitly instructed.
- Push or open a pull request only when requested.

## Review handoff

At completion, provide:
1. Branch name and starting commit.
2. Summary of behavior changed and why.
3. Commit list and main files changed.
4. Validation performed, including failures or checks not run.
5. Remaining limitations and decisions needing review.
6. Any uncommitted changes.