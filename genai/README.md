# GA1 - Natural Language to ROS2 Parameters

UCSD ECE148 Team 13 - Slalomkings

## Overview

GA1 converts English driving instructions into a ROS2 YAML parameter file using the Google Gemini API.

Pipeline:

Natural Language -> Gemini API -> JSON -> Python Validation -> ROS2 YAML

The tool runs independently of ROS2 and does not require access to the car.

## Requirements

- Python 3.11+
- Google Gemini API key
- Python packages listed in requirements.txt

Tested environment:
- Python 3.11.14
- google-genai 3.6.1
- PyYAML 6.0.4
- pytest 9.2.0

Model: gemini-3.5-flash-lite

## Installation (Windows PowerShell)

From the Slalomkings repository root:

```powershell
py -3.11 -m venv ..\slalom-genai-venv
..\slalom-genai-venv\Scripts\python.exe -m pip install -r genai\requirements.txt
```

The virtual environment is kept outside the Git repository.

## API Key

Create an API key at:
https://aistudio.google.com/apikey

Set GEMINI_API_KEY in your current PowerShell session.

For hidden input:

```powershell
$secure = Read-Host "Gemini API key" -AsSecureString
$ptr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($secure)
try {
    $env:GEMINI_API_KEY = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($ptr)
}
finally {
    [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($ptr)
    Remove-Variable secure,ptr -ErrorAction SilentlyContinue
}
```

Never commit, print, or share the API key.

## Usage

Run from the repository root:

```powershell
..\slalom-genai-venv\Scripts\python.exe genai\generate.py "Start on the right, set throttle to 0.27"
```

Alternatively, run without a prompt argument to enter instructions interactively.

The tool generates:

```text
Slalomkings/genai.yaml
```

Each successful run overwrites this generated YAML file.

## Supported Parameters

| Parameter | Default | Allowed Range |
|---|---|---|
| first_side_left | true | boolean |
| throttle | 0.3 | 0.20-0.40 |
| offset | 0.4 | 0.30-0.60 |
| stop_dist | 0.7 | 0.50-1.00 |
| search_steer | 0.8 | 0.50-0.90 |
| pass_max_x | 1.5 | 1.00-1.80 |
| clear_frames | 18 | integer 10-25 |

Only these seven parameters are supported.

Unspecified parameters retain their defaults. Numeric values outside the allowed ranges are clamped.

Unsupported parameter names, invalid types, non-finite numbers, duplicate JSON keys, and invalid responses are rejected.

## Real Gemini API Examples

The following results were generated using gemini-3.5-flash-lite during testing on November 23, 2026.

### Example 1: Explicit parameters

Prompt:

"Start on the right, set throttle to 0.27, 4 yellow cones 2 m apart."

LLM changes:

```json
{"first_side_left": false, "throttle": 0.27}
```

Generated YAML:

```yaml
driver:
  ros__parameters:
    first_side_left: false
    throttle: 0.27
    offset: 0.4
    stop_dist: 0.7
    search_steer: 0.8
    pass_max_x: 1.5
    clear_frames: 18
```

### Example 2: Qualitative speed adjustment

Prompt:

"Start on the left, drive a bit slower, stop at the pink cone."

LLM changes:

```json
{"first_side_left": true, "throttle": 0.25}
```

Generated YAML:

```yaml
driver:
  ros__parameters:
    first_side_left: true
    throttle: 0.25
    offset: 0.4
    stop_dist: 0.7
    search_steer: 0.8
    pass_max_x: 1.5
    clear_frames: 18
```

### Example 3: No supported parameter changes

Prompt:

"4 yellow cones, 2 m apart. Stop at the pink cone."

LLM changes:

```json
{}
```

Generated YAML:

```yaml
driver:
  ros__parameters:
    first_side_left: true
    throttle: 0.3
    offset: 0.4
    stop_dist: 0.7
    search_steer: 0.8
    pass_max_x: 1.5
    clear_frames: 18
```

Cone count and cone spacing are intentionally ignored because the current Driver does not support these parameters.

## Testing

From the repository root:

```powershell
..\slalom-genai-venv\Scripts\python.exe -m pytest -v genai\tests\test_generate.py
```

Tests cover:
- Parameter defaults and type validation
- Numeric clamping
- Invalid JSON and unsupported parameters
- YAML serialization
- Mock API success and failure
- Three real Gemini API integration cases

Verification result: 53 tests passed.

Real API tests require GEMINI_API_KEY and network access. They are skipped when the key is unavailable.

## ROS2 Integration

Copy genai.yaml to the ROS2 environment.

Run:

```bash
ros2 run slalom driver --ros-args --params-file genai.yaml
```

The YAML root node name is `driver`, matching the existing ROS2 node.

If existing launch scripts also set these parameters using `-p`, coordinate the overlapping overrides with the integration owner.

The GA1 tool does not start the car, modify the Driver, or execute vehicle commands.

## Notes

- Gemini interprets natural language and proposes changes.
- Python independently validates all proposed values.
- Natural-language interpretation is not guaranteed to be perfect.
- API failures produce an error instead of fabricated YAML.
- Vehicle behavior and ROS2 loading must be verified separately by the integration team.

Developed for UCSD ECE148 Team 13.
