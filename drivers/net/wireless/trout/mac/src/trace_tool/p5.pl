#tool functions;
sub get_var_according_formula
{
	my ($var_name) = @_;
	if($var_name =~ /\d+/)			#means the var_name is a const name 
	{
		return $var_name;
	}
	else
	{
		return $g_statistics{$N}->{'var'}->{$var_name};
	}
}
sub check_if_set_valid
{
	my ($formula) = @_;
	my $i = 0;
	my $form_id;
	my $form_type;
	while($i < $formula->{'run_check'}->{'number'})
	{
		########################wait for confirm
		if($formula->{'run_check'}->{$i}->{'valid'} == 1)
		{
			printf("check_if_set_valid\n");
			$form_id=$formula->{'run_check'}->{$i}->{'id'};
			$form_type = &get_formula_type($g_statistics{$N}->{'formula'}->{$form_id}); 
			if($form_type =~ /when/)
			{
				printf("set unvalid,form_id:$form_id,formula_id:$formula_id,i:$i\n");
				$g_statistics{$N}->{'formula'}->{$form_id}->{'range_do'}->{$formula_id} = 1;   #set this formula has done.
				$formula->{'run_check'}->{$i}->{'valid'} = 0;			#close the valid
			}
		}
		$i++;
	}
}
#############################text get#################################
sub get
{
	my ($formula) = @_;
	my $result = get_var_according_formula($formula->{'result'});
	my $var1   = $formula->{'var1'};
	printf("the var1 is :$var1\n");
	if($string =~ /$var1/)
	{
		if($string =~/($var1)\s*=\s*(\d+)/)
		{
			$result ->{'value'} = $2;
			$result ->{'valid'} = 1;
			printf("wanlin:\t$2\n\n");
			check_if_set_valid($formula);
		}
		return 0;
	}
	else
	{
		printf("can not get $var1\n");
		return 0;						#complet
	}
}
##############################end####################################
#############################add#####################################
sub add
{
	my ($formula) = @_;
	my $result =  get_var_according_formula($formula->{'result'});
	my $var1   =  get_var_according_formula($formula->{'var1'});
	my $var2  =  get_var_according_formula($formula->{'var2'});
	$result->{'value'} = $var1->{'value'} + $var2->{'value'};
	$result->{'valid'} = 1;
	$var1->{'valid'} = 0;
	$var2->{'valid'} = 0;
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
#############################reduce##################################
sub reduce
{
	my ($formula) = @_;
	my $result =  get_var_according_formula($formula->{'result'});
	my $var1   =  get_var_according_formula($formula->{'var1'});
	my $var2  =  get_var_according_formula($formula->{'var2'});
	$result->{'value'} = $var1->{'value'} - $var2->{'value'};
	$result->{'valid'} = 1;
	$var1->{'valid'} = 0;
	$var2->{'valid'} = 0;
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
###############################multiply################################
sub multiply
{
	my ($formula) = @_;
	my $result =  get_var_according_formula($formula->{'result'});
	my $var1   =  get_var_according_formula($formula->{'var1'});
	my $var2  =  get_var_according_formula($formula->{'var2'});
	$result->{'value'} = $var1->{'value'} * $var2->{'value'};
	$result->{'valid'} = 1;
	$var1->{'valid'} = 0;
	$var2->{'valid'} = 0;
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
############################division###################################
sub division
{
	my ($formula) = @_;
	my $result =  get_var_according_formula($formula->{'result'});
	my $var1   =  get_var_according_formula($formula->{'var1'});
	my $var2  =  get_var_according_formula($formula->{'var2'});
	if($var2 == 0)
	{
		printf("Dividend cann't be zero\n");
		return -1;
	}
	$result->{'value'} = $var1->{'value'} / $var2->{'value'};
	$result->{'valid'} = 1;
	$var1->{'valid'} = 0;
	$var2->{'valid'} = 0;
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
#####################################################################
#				  R=average(A,N)
############################average###################################
sub get_one_var
{
	my ($var1,$coordinate,$coordinate_temp) = @_;
	my $coordinate_tmp_xy =  $coordinate_temp;
	my $coordinate_xy	=  $coordinate;
	my $coordinate_tmp_xy_num = $coordinate_tmp_xy->{'number'};
	$coordinate_tmp_xy->{$coordinate_tmp_xy_num} =$var1->{'value'};
	$coordinate_tmp_xy->{'number'} = $coordinate_tmp_xy->{'number'} + 1;
}
sub calculate_and_store
{
	my ($type,$coordinate,$coordinate_temp) = @_;
	my $coordinate_tmp_xy =  $coordinate_temp;
	my $coordinate_xy	=  $coordinate;
	my $coordinate_xy_num = $coordinate_xy->{'number'};
	my $coordinate_tmp_xy_num = $coordinate_tmp_xy->{'number'};
	my $i=0;
	my $temp = 0;
	while($i < $coordinate_tmp_xy_num)
	{
		$temp += $coordinate_tmp_xy->{$i};
		$coordinate_tmp_xy->{$coordinate_tmp_xy_num} = 0;
		$i++;
	}
	if($type == 0)											#for average
	{
		$temp/=$coordinate_tmp_xy_num;
	}
	elsif($type == 1)										#for sum
	{
	}
	$coordinate_tmp_xy->{'number'} = 0;
	$coordinate_xy->{$coordinate_xy_num} =$temp;
	$coordinate_xy->{'number'} = $coordinate_xy->{'number'} + 1;
}
$g_act_average_count = 0;
$g_cur_act_average_count = 0;
$g_act_num = 0;
sub average
{
	my ($formula) = @_;
	my $result =  get_var_according_formula($formula->{'result'});
	my $var1   =  get_var_according_formula($formula->{'var1'});		#A
	my $var2  =  get_var_according_formula($formula->{'var2'});		#N
	my $result_temp;
	if($formula->{'result'} =~ /xx/)
	{
		$result_temp = get_var_according_formula('coordinate_temp_xx');
	}
	else
	{
		$result_temp = get_var_according_formula('coordinate_temp_yy');
	}
	if($var2 == 0)
	{
		get_one_var($var1,$result,$result_temp);
	}
	elsif($g_act_average_count == 0)
	{
		$g_act_average_count = $var2;
		$g_cur_act_average_count = $var2;
	}
	else
	{
		get_one_var($var1,$result,$result_temp);
		$g_cur_act_average_count--;
		if($g_cur_act_average_count == 0)
		{
			calculate_and_store(0,$result,$result_temp);
		}
	}
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
#####################################################################
#				  R=average(A,N)
############################average###################################
$g_act_sum_count = 0;
$g_cur_act_sum_count = 0;
sub sum
{
	my ($formula) = @_;
	my $result =  get_var_according_formula($formula->{'result'});
	my $var1   =  get_var_according_formula($formula->{'var1'});		#A
	my $var2  =  get_var_according_formula($formula->{'var2'});		#N
	my $result_temp;
	if($formula->{'result'} =~ /xx/)
	{
		$result_temp = get_var_according_formula('coordinate_temp_xx');
	}
	else
	{
		$result_temp = get_var_according_formula('coordinate_temp_yy');
	}
	if($var2 == 0)
	{
		get_one_var($var1,$result,$result_temp);
	}
	elsif($g_act_average_count == 0)
	{
		$g_act_average_count = $var2;
		$g_cur_act_average_count = $var2;
	}
	else
	{
		get_one_var($var1,$result,$result_temp);
		$g_cur_act_average_count--;
		if($g_cur_act_average_count == 0)
		{
			calculate_and_store(1,$result,$result_temp);
		}
	}
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
##########################when_define#################################
sub set_condition_match
{
	my ($clouse_num) =@_;
	my $i = 0;
	printf("rely on line number:$clouse_num\n");
		while($i < $clouse_num)
		{
			printf("n:$N,formula_id:$formula_id\n");
			$g_statistics{$N}->{'formula'}->{$i}->{'run_check'}->{$formula_id}->{'valid'} = 1;
			$g_statistics{$N}->{'formula'}->{$i}->{'run_check'}->{$formula_id}->{'id'} = $N;
			$g_statistics{$N}->{'formula'}->{$form_id}->{'range_do'}->{$i} = 1;
			$i++;
		}
}
######################################################################
sub when_define
{
	my ($formula) = @_;
	print("wanlin:fdfdfdf\n");
	my $var1   = $formula->{'var1'};
	my $startline = $formula->{'result'};
	my $endline  = $formula->{'var3'};
	printf("startline:$startline,endline:$endline\n");
	my $clouse_num = $endline - $endline + 1;
	if($string =~ /$var1/)
	{
		printf('the conditin is match now'."\n");
		$clouse_num =5;						#fortest
		set_condition_match($clouse_num);
		return 0;
	}
	else
	{
		return 0;						#complete
	}
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
##########################when_xx#################################
sub when_xx
{
	my ($formula,$type) = @_;
	my $var1   =  get_var_according_formula($formula->{'var1'});
	my $var2  =  get_var_according_formula($formula->{'var2'});
	my $startline = $formula->{'result'};
	my $endline  = $formula->{'var3'};
	my $clouse_num = $endline - $endline + 1;
	if($type eq 'eq')
	{
		if($var1->{'value'} ==$var2->{'value'})
		{
			set_condition_match($clouse_num);
		}
	}
	elsif($type eq 'ne')
	{
		if($var1->{'value'} !=$var2->{'value'})
		{
			set_condition_match($clouse_num);
		}
	}
	elsif($type eq 'gt')
	{
		if($var1->{'value'}  > $var2->{'value'})
		{
			set_condition_match($clouse_num);
		}		
	}
	elsif($type eq 'ge')
	{
		if($var1->{'value'}  >= $var2->{'value'})
		{
			set_condition_match($clouse_num);
		}				
	}
	elsif($type eq 'le')
	{
		if($var1->{'value'}  <= $var2->{'value'})
		{
			set_condition_match($clouse_num);
		}				
	}
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
#####################################################################
sub set_condition_match_after
{
	my ($clouse_num) =@_;
	my $i = 0;
		while($i < $clouse_num)
		{
			$g_statistics{$N}->{'formula'}->{$i}->{'run_check'}->{$N}->{'valid'} = 1;
			$g_statistics{$N}->{'formula'}->{$i}->{'run_check'}->{$N}->{'id'} = $N;
			$i++;
		}
}
sub after_define
{
	my ($formula) = @_;
	my $var1   = $formula->{'var1'};
	my $startline = $formula->{'result'};
	my $endline  = $formula->{'var3'};
	my $clouse_num = $endline - $endline + 1;
	if($string =~ /$var1/)
	{
		set_condition_match_after($clouse_num);
		return 0;
	}
	else
	{
		return 0;						#complete
	}
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
sub after_xx
{
	my ($formula,$type) = @_;
	my $var1   =  get_var_according_formula($formula->{'var1'});
	my $var2  =  get_var_according_formula($formula->{'var2'});
	my $startline = $formula->{'result'};
	my $endline  = $formula->{'var3'};
	my $clouse_num = $endline - $endline + 1;
	if($type eq 'eq')
	{
		if($var1->{'value'} ==$var2->{'value'})
		{
			set_condition_match_after($clouse_num);
		}
	}
	elsif($type eq 'ne')
	{
		if($var1->{'value'} !=$var2->{'value'})
		{
			set_condition_match_after($clouse_num);
		}
	}
	elsif($type eq 'gt')
	{
		if($var1->{'value'}  > $var2->{'value'})
		{
			set_condition_match_after($clouse_num);
		}		
	}
	elsif($type eq 'ge')
	{
		if($var1->{'value'}  >= $var2->{'value'})
		{
			set_condition_match_after($clouse_num);
		}				
	}
	elsif($type eq 'le')
	{
		if($var1->{'value'}  <= $var2->{'value'})
		{
			set_condition_match_after($clouse_num);
		}				
	}
	if($formula->{'run_check'}->{'number'} = 0)
	{
		return 0;
	}
	else
	{
		check_if_set_valid($formula);
		return 0;
	}
}
#############################end#####################################
sub get_formula_type
{
	my ($formula) = @_;
	printf("int get_formula_type\n");
	return $formula->{'action'};
}
sub execute_formula_type
{
	my ($formula,$formula_type) =@_;
	#printf("wanlin:search formula_type\n");
	if($formula_type eq 'get')
	{
		printf("wanlin:search formula_type\n");	
		get($formula);
	}
	elsif($formula_type eq 'add')
	{
		add($formula);
	}
	elsif($formula_type eq 'reduce')
	{
		reduce($formula);
	}
	elsif($formula_type eq 'multiply')
	{
		multiply($formula);
	}
	elsif($formula_type eq 'division')
	{
		division($formula);
	}
	elsif($formula_type eq 'average')
	{
		average($formula);
	}
	elsif($formula_type eq 'sum')
	{
		sum($formula);
	}
	elsif($formula_type eq 'when_define')
	{
		when_define($formula);
	}
	elsif($formula_type eq 'when_eq')
	{
		when_eq($formula);
	}
	elsif($formula_type eq 'when_ne')
	{
		when_xx($formula,'ne');
	}
	elsif($formula_type eq 'when_gt')
	{
		when_xx($formula,'gt');
	}
	elsif($formula_type eq 'when_lt')
	{
		when_xx($formula,'lt');
	}
	elsif($formula_type eq 'when_ge')
	{
		when_xx($formula,'ge');
	}
	elsif($formula_type eq 'when_le')
	{
		when_xx($formula,'le');
	}
	elsif($formula_type eq 'after_define')
	{
		after_define($formula);
	}
	elsif($formula_type eq 'after_eq')
	{
		after_xx($formula,'eq');
	}
	elsif($formula_type eq 'after_ne')
	{
		after_xx($formula,'ne');
	}
	elsif($formula_type eq 'after_gt')
	{
		after_xx($formula,'gt');
	}
	elsif($formula_type eq 'after_lt')
	{
		after_xx($formula,'after_lt');
	}
	elsif($formula_type eq 'after_ge')
	{
		after_xx($formula,'ge');
	}
	elsif($formula_type eq 'after_le')
	{
		after_xx($formula,'le');
	}
}
sub formula_execute
{
	local ($N,$formula_id,$string) = @_;
	my $result;
	my $var_check_valid = 0;
	my $formula_type;
	printf("===================================================\n");
	printf("wanlin mark".$string."\n");
	$formula = $g_statistics{$N}->{'formula'}->{$formula_id};
	$formula_type = get_formula_type($formula);
	printf("formula_type is:$formula_type\n");
	$result = execute_formula_type($formula,$formula_type);
}



1;
