require 'p5.pl';
sub get_var_valid
{
	my ($var) = @_;
	$var = get_var_according_formula($var);
	unless ($var =~ /\w/)
	{
		return 0;
	}
	if($var->{'valid'} == 1)
	{
		return 0;
	}
	return -1;
}
sub var_check
{
	my ($N,$formula_id) = @_;
	my $valid_var1;
	my $valid_var2;
	my $valid_var3;
	my $formula = $statistics{$N}->{'formula'}->{$formula_id};
	my $form_type = get_formula_type($formula);
	if($form_type eq 'when_define' || $form_type eq 'after_define' || $form_type eq 'get')
	{
		return 0;
	}
	else
	{
		if(get_var_valid($formula->{'var1'}) == 0 && get_var_valid($formula->{'var2'}) == 0)
		{
			return 0;
		}
		else
		{
			return -1;
		}	
	}
}
sub condition_check
{
	my ($formula) = @_;
	my $i = 0;
	my $condition_check_valid = 0;
#########################if the condition rely on is empty#####################
	if($formula->{'run_check'}->{'number'} == 0)
	{
		return 0;
	}
#####################else check the valid of the rely condintions##############
	while($i < $formula->{'run_check'}->{'number'})
	{
		printf("in check    i:$i,$formula->{'run_check'}->{$i}->{'valid'}\n");
		if($formula->{'run_check'}->{$i}->{'valid'} >= 1)
		{
			$condition_check_valid++;
		}
		$i++;
	}
	if($formula->{'run_check'}->{'number'} == $condition_check_valid)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}
sub formula_valid_check
{
	my ($N,$formula_id) = @_;
	@_ = ();
	my $result;
	my $var_check_valid = 0;
	my $formula = $g_statistics{$N}->{'formula'}->{$formula_id};
	$result = condition_check($formula);
	if($result == -1)
	{	
		printf("check found not match1\n");
		return 0;
	}
	$result = var_check($N,$formula_id);
	if($result == -1)
	{
		printf("check found not match2\n");
		return 0;
	}
	printf("ok check ok\n");
	#ok condition match;
	return 1;
}


1;
