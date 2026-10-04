$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")
if (-not (Test-Path "dashboard/frontend/dist/index.html")) {
    Push-Location dashboard/frontend
    npm ci
    if ($LASTEXITCODE -ne 0) { throw "npm ci failed" }
    npm run build
    if ($LASTEXITCODE -ne 0) { throw "Frontend build failed" }
    Pop-Location
}
python -m uvicorn dashboard.backend.app:app --host 127.0.0.1 --port 8000
