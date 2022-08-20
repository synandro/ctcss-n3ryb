#!/usr/bin/perl -wT

use Math::BigFloat;
#my @rcommon_values = (10, 20, 22, 33, 47, 51, 100, 120, 150, 200, 220, 240, 270, 300, 330, 470, 510, 560, 680, 820, 1000, 1200, 1500, 2000, 2200, 2400, 2700, 3000, 3300, 4700, 5100, 6800);
my @rcommon_values = (20000, 22000, 30000, 33000, 47000, 51000, 68000, 100000, 150000,200000, 220000, 300000, 330000, 390000, 470000, 680000, 1000000, 2200000, 4700000, 9600000);
#my @rcommon_values = ();
#my @rcommon_values = (68000, 100000, 200000, 222000, 300000, 330000, 390000, 470000, 680000, 1000000, 2200000, 4700000, 96000000);
#my @rcommon_values = ();
#my @common_values = (100000, 200000, 300000, 400000, 500000, 600000, 700000, 800000, 900000, 1000000, 1100000, 1200000, 1300000, 1400000, 1500000, 1600000, 1700000, 1800000, 1900000, 2000000, 2100000, 2200000, 2300000);
#my @common_values = (390000, 780000, 1170000, 1560000, 1950000, 2340000, 2730000, 3120000, 3510000, 3900000, 4290000);

my @common_values = (91000, 100000, 120000, 180000, 200000, 300000, 360000, 390000, 510000, 820000, 1500000,2000000,3000000);
#my @c = (100000, 120000, 360000, 390000, 510000, 300000, 91000, 820000, 820000,180000, 910000, 2000000, 200000, 560000);
#my @common_values = sort { $a cmp $b } @c;
#my @rcommon_values = 


my $fixed_voltage = 8.0;
# my $fixed_r2 = 10;
sub do_div
{
	my ($r1, $r2, $vs) = @_;
	return ($vs * $r2) / ($r1 + $r2);
}


# i can only vary R1, R2 is fixed
my $last_index = 0;
my @output;
	my $q = 0;
foreach my $fixed_r2 (@rcommon_values)
{
	$q = 0;
#	$fixed_r2 += 1000000;
	foreach my $try_r1 (@common_values)
	{
		my $vout = do_div($try_r1, $fixed_r2, $fixed_voltage);
		next if($vout > 5);
		my $index = int 1024/(5/$vout);
		my $diff = $last_index - $index;
		$last_index = $index;
		print "INDEX=$index DIFF=$diff R2=$fixed_r2 R1=$try_r1 vout = $vout index= $index q=$q\n";
		$q++;
		if($q > 13)
		{
			goto out;
		}
	
	}
	out:
	print "\n";
}