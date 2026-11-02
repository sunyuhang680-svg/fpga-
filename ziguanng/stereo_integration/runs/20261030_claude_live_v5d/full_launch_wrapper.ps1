$ErrorActionPreference='Continue'
$j=Get-Content 'D:\虚拟C盘\ziguanng\stereo_integration\runs\20261030_claude_live_v5d\launch_args_v5d.json' -Raw -Encoding utf8 | ConvertFrom-Json
$a=@($j.full); $exe=$a[0]; $rest=$a[1..($a.Count-1)]
Set-Location $j.cwd
& $exe @rest *>> 'D:\虚拟C盘\ziguanng\stereo_integration\runs\20261030_claude_live_v5d\full_launch.log'
"EXITCODE=$LASTEXITCODE" | Out-File -Append -Encoding utf8 'D:\虚拟C盘\ziguanng\stereo_integration\runs\20261030_claude_live_v5d\full_launch.log'