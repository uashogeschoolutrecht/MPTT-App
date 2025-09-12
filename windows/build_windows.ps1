# Build Windows executable with PyInstaller and package with Inno Setup
param(
  [string]$Python = "python",
  [string]$Spec = "pyinstaller-win.spec",
  [string]$InnoSetup = "C:\\Program Files (x86)\\Inno Setup 6\\ISCC.exe"
)

# Create venv and install deps if needed (optional, comment out if you manage env elsewhere)
# & $Python -m venv .venv
# & .venv\\Scripts\\pip install -r requirements_win.txt

# Build app via PyInstaller
& $Python -m PyInstaller --noconfirm $Spec
if ($LASTEXITCODE -ne 0) { throw "PyInstaller failed" }

# Build installer via Inno Setup
if (Test-Path $InnoSetup) {
  & "$InnoSetup" windows\\installer.iss
  if ($LASTEXITCODE -ne 0) { throw "Inno Setup failed" }
} else {
  Write-Host "Inno Setup not found at $InnoSetup. Install from https://jrsoftware.org/isinfo.php or adjust path."
}
