; Inno Setup script for MPPT-App
#define MyAppName "MPPT-App"
#define MyAppVersion "1.0.0"
#define MyAppPublisher "HU"
#define MyAppURL "https://github.com/uashogeschoolutrecht/Spiral-Tracking-App"
#define MyAppExeName "MPPT-App.exe"

[Setup]
AppId={{7B8C9F5E-8C12-4F3F-86F7-7B8C9F5E8C12}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
DefaultDirName={autopf64}\{#MyAppName}
DisableDirPage=no
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
OutputBaseFilename=MPPT-App-Setup
Compression=lzma
SolidCompression=yes
WizardStyle=modern
SetupIconFile=assets\appicon.ico
ArchitecturesInstallIn64BitMode=x64

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Files]
; Include all files from the PyInstaller dist folder for Windows
Source: "dist\MPPT-App\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop icon"; GroupDescription: "Additional icons:"; Flags: unchecked

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "Launch {#MyAppName}"; Flags: nowait postinstall skipifsilent
