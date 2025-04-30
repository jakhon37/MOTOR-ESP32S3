To manage multiple versions of your code on GitHub, including a `main` branch for stable releases and a `dev` branch for development, and to create new versions as needed, you can use Git branches and tags effectively. Below is a step-by-step guide to achieve this, based on your current setup where you’ve already pushed your code to the `main` branch.

### Step-by-Step Guide

#### 1. **Understand Your Current State**
- Your repository `MOTOR-ESP32S3` has a `main` branch, and you’ve successfully pushed your initial code to it (`origin/main`).
- The `git status` output shows your working tree is clean, meaning there are no uncommitted changes.
- You want to:
  - Keep `main` as the stable branch for production-ready code.
  - Create a `dev` branch for ongoing development.
  - Manage different versions (e.g., `v1.0`, `v2.0`) for releases.

#### 2. **Create a Development Branch (`dev`)**
The `dev` branch will be where you make changes and test new features before merging them into `main`.

```bash
# Create and switch to a new dev branch
git checkout -b dev

# Push the dev branch to GitHub
git push -u origin dev
```

- This creates a `dev` branch that starts from the current state of `main`.
- The `-u` flag sets up tracking so `git push` and `git pull` work without specifying the remote branch.

#### 3. **Set `main` as the Default Branch (Optional, Already Done in Your Case)**
Since you’ve already pushed to `main`, it’s likely the default branch on GitHub. To confirm or set `main` as the default:
- Go to your repository on GitHub (`https://github.com/jakhon37/MOTOR-ESP32S3`).
- Navigate to the **Settings** tab.
- Under **Branches**, ensure `main` is set as the default branch.

#### 4. **Work on the `dev` Branch**
Make changes to your code in the `dev` branch. For example:
- Edit files, add new features, or fix bugs.
- Commit your changes:

```bash
# Add changed files
git add .

# Commit changes with a message
git commit -m "Add new feature for version 2.0"

# Push changes to the dev branch on GitHub
git push origin dev
```

#### 5. **Create a New Version (Release)**
When your changes in `dev` are stable and ready for release, you can merge them into `main` and tag the release with a version number (e.g., `v1.0`).

##### a. Merge `dev` into `main`
```bash
# Switch to the main branch
git checkout main

# Merge dev into main
git merge dev

# Resolve any merge conflicts if they occur (Git will guide you)
# After resolving conflicts, commit the merge
git commit

# Push the updated main branch to GitHub
git push origin main
```

##### b. Tag the Release
Tagging creates a reference to a specific commit for a version (e.g., `v1.0`).

```bash
# Create a tag for the current commit on main
git tag v1.0

# Push the tag to GitHub
git push origin v1.0
```

- You can view releases on GitHub under the **Releases** tab in your repository.
- Optionally, create a release on GitHub for `v1.0`:
  - Go to your repository on GitHub.
  - Click **Releases** > **Create a new release**.
  - Select the `v1.0` tag, add a title (e.g., "Version 1.0"), and describe the changes.

#### 6. **Continue Development on `dev`**
After merging and tagging, switch back to `dev` to continue working on the next version (e.g., `v2.0`).

```bash
# Switch to dev branch
git checkout dev

# Make further changes, commit, and push
git add .
git commit -m "Start working on version 2.0 features"
git push origin dev
```

#### 7. **Create Additional Branches for Other Versions (Optional)**
If you need to work on specific versions or features in parallel (e.g., a bugfix for `v1.0` or a new feature for `v2.0`), create feature or version-specific branches from `main` or `dev`.

For example, to fix a bug in `v1.0`:
```bash
# Create a branch from the v1.0 tag
git checkout v1.0
git checkout -b bugfix/v1.0.1

# Make changes, commit, and push
git add .
git commit -m "Fix bug in v1.0"
git push -u origin bugfix/v1.0.1
```

To merge the bugfix into `main` and `dev`:
```bash
# Merge into main
git checkout main
git merge bugfix/v1.0.1
git push origin main

# Merge into dev
git checkout dev
git merge bugfix/v1.0.1
git push origin dev

# Tag the bugfix release
git checkout main
git tag v1.0.1
git push origin v1.0.1
```

#### 8. **Fix the `credential-osxkeychain` Error**
The error `git: 'credential-osxkeychain' is not a git command` suggests an issue with Git’s credential helper configuration on macOS. Although it didn’t prevent your push, it’s good to resolve it to avoid future issues.

To fix:
```bash
# Check the current credential helper configuration
git config --global credential.helper

# If it’s set to osxkeychain but not working, reset it
git config --global credential.helper osxkeychain
```

If the error persists:
- Ensure Git is up to date:
  ```bash
  brew install git
  ```
- Or, use a different credential helper (e.g., store credentials in a file):
  ```bash
  git config --global credential.helper store
  ```
- Alternatively, use SSH instead of HTTPS for GitHub:
  - Generate an SSH key if you don’t have one:
    ```bash
    ssh-keygen -t ed25519 -C "your_email@example.com"
    ```
  - Add the SSH key to GitHub (Settings > SSH and GPG keys > New SSH key).
  - Update your remote URL to use SSH:
    ```bash
    git remote set-url origin git@github.com:jakhon37/MOTOR-ESP32S3.git
    ```

#### 9. **Best Practices for Versioning**
- **Semantic Versioning**: Use version numbers like `vX.Y.Z` (e.g., `v1.0.0`):
  - `X`: Major changes (breaking changes).
  - `Y`: Minor changes (new features, backward-compatible).
  - `Z`: Patch (bug fixes, backward-compatible).
- **Protect the `main` Branch**:
  - On GitHub, go to Settings > Branches > Add branch protection rule.
  - Enable “Require pull request reviews before merging” for `main` to ensure changes are reviewed.
- **Use Pull Requests**:
  - Instead of merging `dev` directly into `main`, create a pull request on GitHub for code review and discussion.
  - Example:
    ```bash
    git checkout dev
    git push origin dev
    ```
    Then, on GitHub, create a pull request from `dev` to `main`.

#### 10. **Verify Your Setup**
- Check your branches locally:
  ```bash
  git branch
  ```
- Check branches on GitHub:
  ```bash
  git ls-remote --heads origin
  ```
- View tags:
  ```bash
  git tag
  ```

### Example Workflow Summary
1. **Initial Setup**:
   - `main` branch: Stable code (`v1.0` tagged after merging).
   - `dev` branch: Ongoing development for `v2.0`.
2. **Development**:
   - Work on `dev`, commit, and push changes.
3. **Release**:
   - Merge `dev` into `main`.
   - Tag the release (e.g., `v1.0`, `v1.0.1`, `v2.0`).
   - Push `main` and tags to GitHub.
4. **Bugfixes or Features**:
   - Create branches like `bugfix/v1.0.1` or `feature/new-module`.
   - Merge into `dev` and `main` as needed.

### Additional Notes
- If you want to forget this conversation or manage chat history, you can:
  - Click the book icon beneath the message referencing this chat and select it from the menu to forget it.
  - Disable memory in the "Data Controls" section of settings.
- If you encounter issues or need help with specific Git commands, let me know, and I can provide more detailed guidance or troubleshoot further.

You’re now set to manage `main`, `dev`, and versioned releases on GitHub! Let me know if you need clarification or additional steps.