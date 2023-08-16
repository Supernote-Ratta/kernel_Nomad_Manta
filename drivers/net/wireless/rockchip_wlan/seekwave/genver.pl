#!/usr/bin/perl -s

use POSIX qw(strftime);
use File::Basename;

$skw_branch = "xw0616_dev";
$skw_version = "2.0.230614.627ee61";

$output = shift;
open (OUTPUT, ">$output") || die "$0 : can't open $output for writing\n";

print OUTPUT "#ifndef __SKW_VERSION_H__\n";
print OUTPUT "#define __SKW_VERSION_H__\n";

print OUTPUT "\n";

print OUTPUT "#define SKW_BRANCH     \"$skw_branch\"\n";
print OUTPUT "#define SKW_VERSION    \"$skw_version\"\n";

print OUTPUT "\n";

print OUTPUT "#endif";

close (OUTPUT);
