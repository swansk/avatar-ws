# AVATAR Tools

## Install pyenv
https://github.com/pyenv/pyenv#basic-github-checkout

```bash
# Define environment variable PYENV_ROOT to point to the path where pyenv repo is cloned and add $PYENV_ROOT/bin to your $PATH for access to the pyenv command-line utility.
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc

# Add pyenv init to your shell to enable shims and autocompletion. Please make sure eval "$(pyenv init -)" is placed toward the end of the shell configuration file since it manipulates PATH during the initialization.
echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.bashrc

# Restart Shell
exec "$SHELL"
```

## Install Python 3.9
```bash
pyenv install 3.9.0
```

## Install Poetry
https://python-poetry.org/docs/#installation
```bash
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python -
```

Reboot now please.

Install poetry dependencies
```bash
poetry install
```
