Param
(
    [Parameter()]
    [string]$filename="data",
    [string]$filetype=".png",
    [string]$source,
    [string]$destination,
    [int]$start,
    [int]$end
)

$val = $start

while ($val -lt $end)
{
    $name = $source + $filename + $val.ToString() + $filetype
    Write-Host $name
    Copy-Item $name $destination
    $val++
}