# Setup guide and team workflow

## 1. One-time setup (Ubuntu 22.04)

```bash
sudo apt update && sudo apt install -y git python3-venv python3-pip
cd ~
git clone https://github.com/<owner>/<repo>.git pluto_swarm
cd pluto_swarm

# Anonymous identity for THIS repo only (competition rule: no names)
git config user.name "Team Drona"
git config user.email "team-drona@users.noreply.github.com"

# Python environment ("venv" = a private box of packages just for this project)
python3 -m venv .venv
source .venv/bin/activate          # do this in every new terminal
pip install -e ".[dev]"            # -e = editable: your code edits apply immediately
pytest -q
```

## 2. Every work session

```bash
cd ~/pluto_swarm && source .venv/bin/activate
git checkout main && git pull                 # get everyone's latest work
git checkout -b control/my-task               # new branch per task: <area>/<task>

# ... edit code (or ask Claude Code) ...
pytest -q                                     # tests must pass
pluto-run --profile sim --mission hover_test  # try it

git add -A
git commit -m "Short description of the change"
git push -u origin control/my-task
```
Then on GitHub: **Compare & pull request** → teammate reviews → **Merge**.

## 3. Rules
- Never commit straight to `main`.
- Small pull requests, often (every day or two).
- Pull `main` before starting anything.
- If `pip install -e .` was done once, you never need to reinstall after editing
  Python files — only after changing `pyproject.toml`.

## 4. Running pieces separately
```bash
python -m sim.plant.world --profile sim            # terminal 1: emulator only
pluto-run --profile sim --mission hover_test       # terminal 2: but set sim.spawn: false first
```
