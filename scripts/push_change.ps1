param(
  [string]$RepoPath = "c:\Users\bav12\Documents\Kinematics Project 1 - 20260227",
  [string]$Branch = "main",
  [string]$File = "webapp/src/kinematics/abb120.ts",
  [string]$Message = "Prefer IK solutions within joint limits; return null when none"
)

Set-StrictMode -Version Latest

if (-not (Test-Path $RepoPath)) {
  Write-Error "Repo path not found: $RepoPath"
  exit 1
}

Set-Location $RepoPath

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
  Write-Error "git not found in PATH. Install Git for Windows and reopen the terminal."
  exit 1
}

Write-Host "Repository: $RepoPath"
Write-Host "Branch: $Branch"
Write-Host "File: $File"

# Show status
git status --porcelain

# Stage file
git add -- "$File"

# Commit
$commitResult = git commit -m "$Message" 2>&1
if ($LASTEXITCODE -ne 0) {
  Write-Host $commitResult
  Write-Error "Commit failed or no changes to commit."
  exit 1
}

# Pull with rebase
Write-Host "Pulling latest from origin/$Branch (rebase)..."
git pull --rebase origin $Branch
if ($LASTEXITCODE -ne 0) {
  Write-Error "git pull failed. Resolve conflicts and re-run this script."
  exit 1
}

# Push
Write-Host "Pushing to origin/$Branch..."
git push origin $Branch
if ($LASTEXITCODE -ne 0) {
  Write-Error "git push failed. Check authentication and remote settings."
  exit 1
}

Write-Host "Push complete."
