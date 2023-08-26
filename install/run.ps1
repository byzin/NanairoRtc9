param (
    # インスタンス0のアドレス
    [Parameter(Mandatory=$true)]
    [string]$instAddress0,

    # インスタンス1のアドレス
    [Parameter(Mandatory=$true)]
    [string]$instAddress1
)

$Env:VK_SDK_PATH = '.\vulkan'
$Env:VULKAN_SDK = '.\vulkan'
$Env:VK_LAYER_PATH = '.\vulkan\Bin'
python run.py ${instAddress0} ${instAddress1}
