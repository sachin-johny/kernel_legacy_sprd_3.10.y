#use strict;
#use warnings;

#my %g_statistics = ();


sub run_check_valid
{
	my ($statistics_id,$formula_id) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	my $run_check_num = $$formula_hash->{'run_check'}->{'number'};
	for(my $i = 0; $i < $run_check_num; $i++)
	{
		if($$formula_hash->{'run_check'}->{$i}->{'valid'} eq 0)
		{
			return 0;
		}
	}
	return 1;
}


sub range_do_valid
{
	my ($statistics_id,$formula_id) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	my $run_check_num = $$formula_hash->{'run_check'}->{'number'};
	if($run_check_num > 0)
	{
		my $condition_id = $$formula_hash->{'run_check'}->{$run_check_num-1}->{'id'};
		my $condition_hash = \$g_statistics{$statistics_id}->{'formula'}->{$condition_id};
		if($$condition_hash->{'range_do'}->{$formula_id-$condition_id-2} eq 0)
		{
			return 0;
		}
	}
	return 1;

}


sub variable_valid
{
	my ($statistics_id,$formula_id) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	if($$formula_hash->{'action'} eq 'add'
		or $$formula_hash->{'action'} eq 'reduce'
		or $$formula_hash->{'action'} eq 'multiply'
		or $$formula_hash->{'action'} eq 'division')
	{
		if(variable_check($statistics_id,$formula_id,1) eq 0)
		{
			return 0;
		}
	}
	elsif(($$formula_hash->{'action'} =~ /when/
		or $$formula_hash->{'action'} =~ /after/)
		and $$formula_hash->{'action'} !~ /define/)
	{
		if(variable_check($statistics_id,$formula_id,0) eq 0)
		{
			return 0;
		}
	
	}
	return 1;

}


sub variable_check
{
	my ($statistics_id,$formula_id,$type) = @_;
	@_ = ();

	my @var = ();
	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	push @var, $$formula_hash->{'var1'};
	push @var, $$formula_hash->{'var2'};
	foreach my $tmp_var(@var)
	{
		if($tmp_var =~ /[a-zA-Z]/)
		{
			if($type eq 1)
			{
				my $result_name = $$formula_hash->{'result'};
				if($tmp_var eq $result_name)
				{
					next;
				}
				my $result_num = 0;
				my $var_num = 0;
				if($result_name eq 'coordinate_xx' or $result_name eq 'coordinate_yy')
				{
					$result_num = $g_statistics{$statistics_id}->{'var'}->{$result_name}->{'number'};
					if($tmp_var eq 'coordinate_xx' or $tmp_var eq 'coordinate_yy')
					{
						$var_num = $g_statistics{$statistics_id}->{'var'}->{$result_name}->{'number'};
						if($result_num + 1 eq $var_num)
						{
							next;
						}
						else
						{
							return 0;
						}
					}
				}
			}
			if($g_statistics{$statistics_id}->{'var'}->{$tmp_var}->{'valid'} eq 0)
			{
				return 0;
			}
		}

	}
	return 1;
	
}


sub formula_valid_check
{
	my ($statistics_id,$formula_id) = @_;
	@_ = ();

	my $result;
	$result = run_check_valid($statistics_id,$formula_id);
	if($result eq 0)
	{	
		#print("run check is not valid ($statistics_id,$formula_id)\n");
		return 0;
	}
	$result = range_do_valid($statistics_id,$formula_id);
	if($result eq 0)
	{
		#print("range do is not valid ($statistics_id,$formula_id)\n");
		return 0;
	}
	$result = variable_valid($statistics_id,$formula_id);
	if($result eq 0)
	{
		#print("variable is not valid ($statistics_id,$formula_id)\n");
		return 0;
	}
	return 1;

}


sub formula_execute
{
	my ($statistics_id,$formula_id,$log) = @_;
	@_ = ();

	my $action = $g_statistics{$statistics_id}->{'formula'}->{$formula_id}->{'action'};
	if($action eq 'get')
	{
		return formula_execute_get($statistics_id,$formula_id,$log);
	}
	elsif($action =~ /when/ or $action =~ /after/)
	{
		return formula_execute_condition($statistics_id,$formula_id,$log);
	}
	else
	{
		return formula_execute_calculate($statistics_id,$formula_id);
	}

}


sub formula_execute_get
{
	my ($statistics_id,$formula_id,$log) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	my $var_hash = \$g_statistics{$statistics_id}->{'var'};
	my $result_name = $$formula_hash->{'result'};
	my $var1_name = $$formula_hash->{'var1'};
	my $value = 0;
	if($var1_name eq 'CPU_TIME')
	{
		if($log =~ /^\[(\d+)\.(\d+)\]/)
		{
			$value = "$1"."$2";
			$value = int($value);
			#print "<$1><$2><$value>\n"
		}
		else
		{
			print("action: get can't match CPU_TIME at $statistics_id, $formula_id, $log\n");
			return (-1);
		}
	}
	else
	{
		if($log =~ /$var1_name\s*=\s*(\d+)/)
		{
			$value = $1;
		}
		else
		{
			#print("action: get can't match $var1_name at $statistics_id, $formula_id, $log\n");
			return 0;
		}
	}
	my $return = set_variable_value($statistics_id,$result_name,$value);
	if($return eq -1)
	{
		return (-1);
	}
	$return = add_range_do($statistics_id,$formula_id);
	return $return;

}


sub formula_execute_calculate
{
	my ($statistics_id,$formula_id) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	my $result_name = $$formula_hash->{'result'};
	if($result_name =~ /[a-zA-Z]/)
	{
		my $var1 = get_variable_value($statistics_id,$$formula_hash->{'var1'});
		my $var2 = get_variable_value($statistics_id,$$formula_hash->{'var2'});
		my $value = 0;
		if($$formula_hash->{'action'} eq 'add')
		{
			$value = $var1 + $var2;
		}
		elsif($$formula_hash->{'action'} eq 'reduce')
		{
			$value = $var1 - $var2;
			#print " $var1 - $var2 = $value\n";
		}
		elsif($$formula_hash->{'action'} eq 'multiply')
		{
			$value = $var1 * $var2;
		}
		elsif($$formula_hash->{'action'} eq 'division')
		{
			$value = $var1 / $var2;
		}
		else
		{
			print("calculate action error at $statistics_id,$formula_id,$$formula_hash->{'action'}\n");
			return (-1);
		}
		close_variable_valid($statistics_id,$$formula_hash->{'var1'});
		close_variable_valid($statistics_id,$$formula_hash->{'var2'});
		my $return = set_variable_value($statistics_id,$result_name,$value);
		if($return eq -1)
		{
			return (-1);
		}
		$return = add_range_do($statistics_id,$formula_id);
		if($return eq -1)
		{
			return (-1);
		}
		return 0;
	}
	else
	{
		print("calculate formula result name is number at $statistics_id, $formula_id, $result_name\n");
		return (-1);
	}

}


sub formula_execute_condition
{
	my ($statistics_id,$formula_id,$log) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	if($$formula_hash->{'action'} =~ /define/)
	{
		my $var = $$formula_hash->{'var1'};
		if($log =~ /$var/)
		{
			my $return = add_run_check($statistics_id,$formula_id);
			if($return eq -1)
			{
				return (-1);
			}
			$return = add_range_do($statistics_id,$formula_id);
			if($return eq -1)
			{
				return (-1);
			}
		}
		return 0;
	}
	else
	elsif($$formula_hash->{'action'} =~ /_eq/)
	{
		my $var1 = $$formula_hash->{'var1'};
		$var1 = get_variable_value($statistics_id,$var1);
		my $var2 = $$formula_hash->{'var2'};
		$var2 = get_variable_value($statistics_id,$var2);
		if($var1 eq $var2)
		{
			my $return = add_run_check($statistics_id,$formula_id);
			if($return eq -1)
			{
				return (-1);
			}
			$return = add_range_do($statistics_id,$formula_id);
			if($return eq -1)
			{
				return (-1);
			}
		}
		return 0;
	}
	else
	{
		print("condition action isn't be defined at $statistics_id,$formula_id,$log\n");
		return (-1);
	}
	
}


sub get_variable_value
{
	my ($statistics_id,$var_name) = @_;
	@_ = ();

	if($var_name =~ /[a-zA-Z]/)
	{
		if($var_name eq 'coordinate_xx' or $var_name eq 'coordinate_yy')
		{
			my $idx = $g_statistics{$statistics_id}->{'var'}->{$var_name}->{'number'};
			return $g_statistics{$statistics_id}->{'var'}->{$var_name}->{$idx-1};
		}
		else
		{
			return $g_statistics{$statistics_id}->{'var'}->{$var_name}->{'value'};
		}
	}
	else
	{
		return $var_name;
	}

}


sub set_variable_value
{
	my ($statistics_id,$var_name,$value) = @_;
	@_ = ();

	if($var_name =~ /[a-zA-Z]/)
	{
		if($var_name eq 'coordinate_xx' or $var_name eq 'coordinate_yy')
		{
			my $idx = $g_statistics{$statistics_id}->{'var'}->{$var_name}->{'number'};
			$g_statistics{$statistics_id}->{'var'}->{$var_name}->{$idx} = $value;
			$g_statistics{$statistics_id}->{'var'}->{$var_name}->{'number'}++;
		}
		else
		{
			$g_statistics{$statistics_id}->{'var'}->{$var_name}->{'valid'} = 1;
			$g_statistics{$statistics_id}->{'var'}->{$var_name}->{'value'} = $value;
		}
	}
	else
	{
		print("result variable is number at $statistics_id,$var_name\n");
		return (-1);
	}
	return 0;

}



sub close_variable_valid
{
	my ($statistics_id, $var_name) = @_;
	@_ = ();

	if($var_name =~ /[a-zA-Z]/)
	{
		if($var_name ne 'coordinate_xx' and $var_name ne 'coordinate_yy')
		{
			$g_statistics{$statistics_id}->{'var'}->{$var_name}->{'valid'} = 0;
		}
	}

}


sub add_run_check
{
	my ($statistics_id, $formula_id) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
	for(my $i = $$formula_hash->{'result'}; $i < $$formula_hash->{'var3'}+1; $i++)
	{
		my $tmp_hash = \$g_statistics{$statistics_id}->{'formula'}->{$i};
		my $number = $$tmp_hash->{'run_check'}->{'number'};
		if($number > 0)
		{
			my $match = 0;
			for(my $j = 0; $j < $number; $j++)
			{
				if($$tmp_hash->{'run_check'}->{$j}->{'id'} eq $formula_id)
				{
					$$tmp_hash->{'run_check'}->{$j}->{'valid'} = 1;
					$match = 1;
					last;
				}
			}
			if($match ne 1)
			{
				print("formula hash run check id unmatch at $statistics_id, $i, $formula_id\n");
				return (-1);
			}
		}
		else
		{
			print("formula hash run check number error at $statistics_id, $i, $number\n");
			return (-1);
		}
	}
	return 0;

}


sub add_range_do
{
	my ($statistics_id, $formula_id) = @_;
	@_ = ();

	my $formula_hash = \$g_statistics{$statistics_id}->{'formula'}->{$formula_id};
=this
	if($$formula_hash->{'action'} =~ /when/ or $$formula_hash->{'action'} =~ /after/)
	{
		for(my $i = $$formula_hash->{'result'}; $i < $$formula_hash->{'var3'}+1; $i++)
		{
			my $tmp_hash = \$g_statistics{$statistics_id}->{'formula'}->{$i};
			my $number = $$tmp_hash->{'run_check'}->{'number'};
			if($number > 0)
			{
				my $match = 0;
				for(my $j = 0; $j < $number; $j++)
				{
					if($$tmp_hash->{'run_check'}->{$j}->{'id'} eq $formula_id)
					{
						$$tmp_hash->{'run_check'}->{$j}->{'valid'} = 1;
						$match = 1;
						last;
					}
				}
				if($match ne 1)
				{
					print("add formula hash run check id unmatch at $statistics_id, $i, $formula_id.\n");
					return (-1);
				}
			}
			else
			{
				print("add formula hash run check number error at $statistics_id, $i, $number.\n");
				return (-1);
			}
		}
	}
=cut
	for(my $i = 0; $i < $$formula_hash->{'run_check'}->{'number'}; $i++)
	{
		my $id = $$formula_hash->{'run_check'}->{$i}->{'id'};
		my $tmp_hash = \$g_statistics{$statistics_id}->{'formula'}->{$id};
		my $range_number = $$tmp_hash->{'var3'} - $$tmp_hash->{'result'} + 1;
		my $idx = $formula_id - $id - 1;
		$$tmp_hash->{'range_do'}->{$idx} = 1;
		#print "add range do ($range_number,$id,$idx)\n";
		my $idx_start = 0;
		if($idx + 1 eq $range_number)
		{
			$idx_start = 0;
			if($$tmp_hash->{'action'} =~ /when/)
			{
				for(my $j = $$tmp_hash->{'result'}; $j < $$tmp_hash->{'var3'}+1; $j++)
				{
					my $j_hash = \$g_statistics{$statistics_id}->{'formula'}->{$j};
					my $match = 0;
					for(my $k = 0; $k < $$j_hash->{'run_check'}->{'number'}; $k++)
					{
						if($$j_hash->{'run_check'}->{$k}->{'id'} eq $id)
						{
							$$j_hash->{'run_check'}->{$k}->{'valid'} = 0;
							$match = 1;
							last;
						}
					}
					if($match ne 1)
					{
						print("reduce formula hash run check id unmatch at $statistics_id, $j, $id\n");
						return (-1);
					}
				}
			}
		}
		else
		{
			$idx_start = $idx + 1;
		}

		for(my $j = $idx_start; $j < $range_number; $j++)
		{
			$$tmp_hash->{'range_do'}->{$j} = 0;
			my $j_id = $id + $j+1;
			my $j_hash = \$g_statistics{$statistics_id}->{'formula'}->{$j_id};
			if($$j_hash->{'action'} =~ /when/)
			{
				for(my $k = $$j_hash->{'result'}; $k < $$j_hash->{'var3'}+1; $k++)
				{
					my $k_hash = \$g_statistics{$statistics_id}->{'formula'}->{$k};
					my $match = 0;
					for(my $m = 0; $m < $$k_hash->{'run_check'}->{'number'}; $m++)
					{	
						if($$k_hash->{'run_check'}->{$m}->{'id'} eq $j_id)
						{
							$$k_hash->{'run_check'}->{$m}->{'valid'} = 0;
							$match = 1;
							last;
						}
					}
					if($match ne 1)
					{
						print("reduce formula hash run check id unmatch at $statistics_id, $k, $j_id\n");
						return (-1);
					}
				}
			}
		}
	}

	return 0;

}



1;
