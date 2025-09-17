COMPILING the kernel_Nomad_Manta:

cd /home/name/build/ 
git clone git@github.com:Supernote-Ratta/kernel_Nomad_Manta.git
git clone git@github.com:Supernote-Ratta/prebuilts_Nomad_Manta.git
mv prebuilts_Nomad_Manta/ prebuilts

cd kernel_Nomad_Manta/

make CC=../prebuilts/clang/host/linux-x86/clang-r383902b/bin/clang LD=../prebuilts/clang/host/linux-x86/clang-r383902b/bin/ld.lld ARCH=arm64 rk3566_ht_eink_Supernote_defconfig android-11.config rk356x_eink.config -j18

make CC=../prebuilts/clang/host/linux-x86/clang-r383902b/bin/clang LD=../prebuilts/clang/host/linux-x86/clang-r383902b/bin/ld.lld ARCH=arm64 rk3566_ht_eink_SupernoteA6X2.img rk3566_ht_eink_Supernote.img rk3566_ht_eink_SupernoteA5X2.img -j18

python -c "import mkMdtbs; mkMdtbs.packMultiDtbs('rk3566_ht_eink_SupernoteA5X2&@_saradc_ch1=350 rk3566_ht_eink_Supernote&@_saradc_ch1=970 rk3566_ht_eink_SupernoteA6X2&@_saradc_ch1=20')"

cp resource.img  all_resource.img

make CC=../prebuilts/clang/host/linux-x86/clang-r383902b/bin/clang LD=../prebuilts/clang/host/linux-x86/clang-r383902b/bin/ld.lld ARCH=arm64 BOOT_IMG=boot_sample.img rk3566_ht_eink_SupernoteA6X2.img rk3566_ht_eink_Supernote.img rk3566_ht_eink_SupernoteA5X2.img