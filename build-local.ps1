#!/usr/bin/env pwsh
# Local build script for Motion Planning for Manipulators

Write-Host "=== Building Motion Planning for Manipulators ===" -ForegroundColor Cyan

# Step 1: Build web demos with Vite
Write-Host "`n[1/4] Building web demos with Vite..." -ForegroundColor Yellow
npm run build
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Vite build failed!" -ForegroundColor Red
    exit 1
}

# Step 2: Render Quarto book
Write-Host "`n[2/4] Rendering Quarto book..." -ForegroundColor Yellow
quarto render
if ($LASTEXITCODE -ne 0) {
    Write-Host "Error: Quarto render failed!" -ForegroundColor Red
    exit 1
}

# Step 3: Copy web demos to build output
Write-Host "`n[3/4] Copying web demos to build output..." -ForegroundColor Yellow
if (Test-Path "dist/web_demos") {
    if (!(Test-Path "_build/html/web_demos")) {
        New-Item -ItemType Directory -Path "_build/html/web_demos" -Force | Out-Null
    }
    Copy-Item -Recurse -Force "dist/web_demos/*" "_build/html/web_demos/"
    Write-Host "  ✓ Web demos copied successfully" -ForegroundColor Green
} else {
    Write-Host "  ✗ dist/web_demos not found!" -ForegroundColor Red
}

# Step 4: Copy models to build output
Write-Host "`n[4/4] Copying models to build output..." -ForegroundColor Yellow
if (Test-Path "models") {
    if (!(Test-Path "_build/html/models")) {
        New-Item -ItemType Directory -Path "_build/html/models" -Force | Out-Null
    }
    Copy-Item -Recurse -Force "models/*" "_build/html/models/"
    Write-Host "  ✓ Models copied successfully" -ForegroundColor Green
} else {
    Write-Host "  ✗ models directory not found!" -ForegroundColor Red
}

Write-Host "`n=== Build completed successfully! ===" -ForegroundColor Green
Write-Host "Output directory: _build/html" -ForegroundColor Cyan
Write-Host "`nTo preview the site, run: quarto preview" -ForegroundColor Cyan
