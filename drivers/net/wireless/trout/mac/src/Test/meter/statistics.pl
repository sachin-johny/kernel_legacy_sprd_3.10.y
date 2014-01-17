use strict;
use warnings;




########### global variable ##########

my $g_config_file = 'cfg.txt';
my $g_log_file = 'log.txt';
my @g_log = ();
my $DEST = 'null';
my $g_dest_file = 'null';
my %g_param_name = ();
my $g_param_value = [];
my $g_param_statistics = [];
my $g_param_statistics_time = [];




########### process ##########
check_content_file();
read_cfg();
main_statistics();
list_statistics();
end("\nStatistics Finish!\n\n");


########### main function ###########

sub check_content_file
{
	if(defined $ARGV[0])
	{
		if(-e $ARGV[0])
		{
			if(!(-d $ARGV[0]))
			{
				$g_dest_file = $ARGV[0];
			}
			else
			{
				end("\nInput parameter is not a content file: $ARGV[0]\n\n");
			}
		}
		else
		{
			end("\nInput parameter is not a exsit file: $ARGV[0]\n\n");
		}
	}
	else
	{
		end("\nMiss dest file!\n\n");
	}
}


sub read_cfg
{
	open(my $CFG, $g_config_file) || end("\nMiss cfg file: $g_config_file\n\n");
	my $cfg_num = 0;
	while(my $tmp_line = <$CFG>)
	{
		$tmp_line =~ s/\t//g;
		$tmp_line =~ s/\s//g;
		if($tmp_line =~ /\[(.+)\]=\[(.*)\]/)
		{
			my $str_name = $1;
			my $str_lel = $2;
			if(defined $str_name)
			{
				$g_param_name{$str_name} = $cfg_num;
				$g_param_value->[$cfg_num][0] = 0;
				$g_param_value->[$cfg_num][1] = 0;
				$g_param_value->[$cfg_num][2] = 0;
				$g_param_value->[$cfg_num][3] = 0;
				$g_param_statistics->[$cfg_num][0] = 0;
				$g_param_statistics->[$cfg_num][1] = 0;
				$g_param_statistics->[$cfg_num][2] = 0;
				$g_param_statistics->[$cfg_num][3] = 0;
				
				my $param_num = 4;
				if(defined $str_lel)
				{
					my @lel_array = split(/,/, $str_lel);
					foreach my $tmp_lel(@lel_array)
					{
						$tmp_lel =~ s/,//g;
						$tmp_lel =~ s/\s//g;
						$tmp_lel =~ s/\t//g;
						if($tmp_lel =~ /^\d+$/)
						{
							$g_param_value->[$cfg_num][$param_num] = $tmp_lel;
							$g_param_statistics->[$cfg_num][$param_num] = 0;
							$g_param_statistics_time->[$cfg_num][$param_num] = 0;
							$param_num += 1;
						}
						$g_param_statistics->[$cfg_num][$param_num] = 0;
						$g_param_statistics_time->[$cfg_num][$param_num] = 0;
					}
				}
				$cfg_num += 1;
			}
		}
	}
	close($CFG);
	if($cfg_num > 0)
	{
		write_log("\n");
		$cfg_num = 1;
		foreach my $tmp_name(keys(%g_param_name))
		{	
			my $tmp_str = "$cfg_num. $tmp_name:  ";
			my $i = $g_param_name{$tmp_name};
			my $tmp_param_num = @{$g_param_value->[$i]};
			if($tmp_param_num > 4)
			{
				for(my $j = 4; $j < $tmp_param_num; $j++)
				{
					$tmp_str .= "$g_param_value->[$i][$j] ";
				}
			}
			else
			{
				$tmp_str .= 'NOTHING';
			}
			write_log("$tmp_str\n\n");
			$cfg_num += 1;
		}
	}
	else
	{
		end("\nCfg request noting or format error!\n\n");
	}
}


sub main_statistics
{
	open(my $DEST, $g_dest_file) || end("\nCan't open file $g_dest_file\n\n");
	while(my $tmp_line = <$DEST>)
	{
		$tmp_line =~ s/\t//g;
		$tmp_line =~ s/\s//g;
		if($tmp_line =~ /(\d+)\(us\).+\(us\)\)\:(.+)$/)
		{
			my $tmp_time = $1;
			my $tmp_name = $2;
			do_statistics($tmp_name, $tmp_time);
		}
	}
}

sub list_statistics
{
	my $list_num = 1;

	write_log("\n\n\nStatistics List:\n\n\n");
	foreach my $tmp_name(keys(%g_param_name))
	{
		my $id = $g_param_name{$tmp_name};
		write_log("$list_num. $tmp_name\n\n");
		write_log("total    =  $g_param_statistics->[$id][0]\n");
		write_log("min      =  $g_param_statistics->[$id][1] (us)\n");
		write_log("max      =  $g_param_statistics->[$id][2] (us)\n");
		my $ret = 0;
		my $ret_time = 0;
		my $total_time = $g_param_statistics->[$id][3];
		if($g_param_statistics->[$id][0] > 0)
		{
			$ret = sprintf("%.3f",$g_param_statistics->[$id][3] / $g_param_statistics->[$id][0]);
		}
		write_log("average  =  $ret (us)\n");
		my $expand_num = @{$g_param_value->[$id]};
		if($expand_num > 4)
		{
			write_log("\n");
			for(my $j = 4; $j < $expand_num; $j++)
			{
				if($g_param_statistics->[$id][0] > 0)
				{
					$ret = sprintf("%.3f",$g_param_statistics->[$id][$j] / $g_param_statistics->[$id][0] * 100);
					$ret = "  (number: $ret%)";
					$ret_time = sprintf("%.3f",$g_param_statistics_time->[$id][$j] / $total_time * 100);
					$ret_time = "  (time: $ret_time%)";
				}
				else
				{
					$ret = '';
					$ret_time = '';
				}

				if($j == 4)
				{
					write_log("0 (us) ~ $g_param_value->[$id][$j] (us)  =   $g_param_statistics->[$id][$j]$ret$ret_time\n");
				}
				else
				{
					write_log("$g_param_value->[$id][$j-1] (us) ~ $g_param_value->[$id][$j] (us)  =   $g_param_statistics->[$id][$j]$ret$ret_time\n");
				}
			}
			if($g_param_statistics->[$id][0] > 0)
			{
				$ret = sprintf("%.3f",$g_param_statistics->[$id][$expand_num] / $g_param_statistics->[$id][0] * 100);
				$ret = "  (number: $ret%)";
				$ret_time = sprintf("%.3f",$g_param_statistics_time->[$id][$expand_num] / $total_time * 100);
				$ret_time = "  (time: $ret_time%)";
			}
			else
			{
				$ret = '';
				$ret_time = '';
			}

			write_log("$g_param_value->[$id][$expand_num-1] (us) ~ >>  =   $g_param_statistics->[$id][$expand_num]$ret$ret_time\n");
		}
		write_log("\n\n\n");
		$list_num += 1;
	}
}


sub write_log
{
	my ($info) = @_;
	@_ = ();

	print $info;
	push @g_log, $info;
}

sub end
{
	my ($info) = @_;
	@_ = ();
	
	print $info;
	push @g_log, $info;
	open(my $LOG, ">$g_log_file");
	foreach my $tmp_log(@g_log)
	{
		print $LOG $tmp_log;
	}
	close($LOG);
	if($DEST ne 'null')
	{
		close($DEST);
	}
	exit;
}

sub do_statistics
{
	my ($name, $this_time) = @_;
	@_ = ();

	foreach my $tmp_param_name(keys(%g_param_name))
	{
		if($name =~ /$tmp_param_name/)
		{
			my $id = $g_param_name{$tmp_param_name};
			$g_param_statistics->[$id][0] += 1;
			if($g_param_statistics->[$id][3] == 0)
			{
				$g_param_statistics->[$id][1] = $this_time;
				$g_param_statistics->[$id][2] = $this_time;
			}
			else
			{
				if($g_param_statistics->[$id][1] > $this_time)
				{
					$g_param_statistics->[$id][1] = $this_time;
				}
				if($g_param_statistics->[$id][2] < $this_time)
				{
					$g_param_statistics->[$id][2] = $this_time;
				}
			}
			$g_param_statistics->[$id][3] += $this_time;
			my $expand_num = @{$g_param_value->[$id]};
			if($expand_num > 4)
			{
				for(my $i = 4; $i < $expand_num; $i++)
				{
					if($this_time < $g_param_value->[$id][$i])
					{
						$g_param_statistics->[$id][$i] += 1;
						$g_param_statistics_time->[$id][$i] += $this_time;
						return;
					}
				}
				$g_param_statistics->[$id][$expand_num] += 1;
				$g_param_statistics_time->[$id][$expand_num] += $this_time;
			}
		}
	}
}

