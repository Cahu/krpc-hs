#!/usr/bin/env perl

use strict;
use warnings;

use JSON;
use Template;


my $usage = <<EOU;
	$0 <service definition file>
EOU


my $servicedef = shift @ARGV;

if (!$servicedef)
{
	print $usage;
	exit 1;
}

open(my $fh, "<", $servicedef)
	or die "Could not open file: $!";

my $json_text = do {
	local $/;
	<$fh>
};

close($fh);

my $json = decode_json($json_text);


my $tt = Template->new()
	or die Template->error(), "\n";


while (my ($serviceName, $def) = each %$json)
{
	my @procs;
	my @enums;
	my @proxys;
	my @exports;
	my %dependencies;

	use Data::Dumper;

	foreach my $enumName (sort keys %{ $def->{enumerations} })
	{
		my $enumDef = $def->{enumerations}{$enumName};

		my @values = map  { $enumName . "'" . $_->{name} }
			(sort { $a->{value} cmp $b->{value} } @{ $enumDef->{values} });

		push @enums, {
			name   => $enumName,
			values => \@values,
			doc    => cleanup_doc($enumDef->{documentation})
		};

		push @exports, "$enumName(..)";
	}

	foreach my $proxyName (sort keys %{ $def->{classes} })
	{
		my $proxyDef = $def->{classes}{$proxyName};

		push @proxys, {
			name => $proxyName,
			doc  => cleanup_doc($proxyDef->{documentation}),
		};

		push @exports, $proxyName;
	}

	foreach my $procName (sort keys %{ $def->{procedures} })
	{
		my $procDef = $def->{procedures}{$procName};

		my ($proc, $deps) = process_procedure($procName, $procDef);

		push @procs, $proc;
		push @exports, $proc->{name};
		push @exports, $proc->{name} . 'Stream' if ($proc->{ret});

		foreach my $d (@$deps)
		{
			# don't add deps to ourself
			if ($d ne "KRPCHS.$serviceName")
			{
				$dependencies{$d} = 1;
			}
		}
	}

	$tt->process(\*DATA, {
		serviceName => $serviceName,
		procs       => \@procs,
		enums       => \@enums,
		proxys      => \@proxys,
		imports     => [ sort keys %dependencies ],
		exports     => \@exports,
	})
		|| die "$Template::ERROR\n";
}


sub process_procedure
{
	my ($rpcName, $procDef) = @_;

	my $procName = $rpcName;

	if ($procName =~ /^(\w+)_(get|set)_(\w+)$/)
	{
		$procName = $2 . ucfirst $1 . ucfirst $3;
	}

	$procName =~ s/_(\w)/uc $1/ge;
	$procName = lcfirst $procName;

	my $doc = cleanup_doc($procDef->{documentation});

	my ($ret,     $retDeps)     = process_return_type($procDef);
	my ($params,  $paramsDeps)  = process_parameters($procDef);

	my %deps = map { $_ => 1 } (@$retDeps, @$paramsDeps);

	return (
		{ name    => $procName
		, rpcname => $rpcName
		, ret     => $ret
		, doc     => $doc
		, params  => $params
		}, 
		[ keys %deps ]
	);
}


sub process_return_type
{
	my ($proc) = @_;

	my $attributes = $proc->{attributes};

	my $ret;
	my %dependencies;

	foreach my $a (@$attributes)
	{
		if ($a =~ /ReturnType\.(.+)/)
		{
			($ret, my $deps) = haskell_type($1);

			foreach my $dep (@$deps)
			{
				$dependencies{$dep} = 1;
			}
		}
	}

	if (not $ret and $proc->{return_type})
	{
		($ret, my $deps) = haskell_type($proc->{return_type});

		foreach my $dep (@$deps)
		{
			$dependencies{$dep} = 1;
		}
	}

	return ($ret, [keys %dependencies])
}


sub process_parameters
{
	my ($proc) = @_;

	my $parameters = $proc->{parameters};
	my $attributes = $proc->{attributes};

	my @params;
	my %dependencies;

	foreach my $a (@$attributes)
	{
		if ($a =~ /ParameterType\((\d+)\)\.(.+)/)
		{
			my $num  = $1;
			my $type = $2;

			my ($t, $deps) = haskell_type($type);

			$params[$num]{type} = $t;

			foreach my $dep (@$deps)
			{
				$dependencies{$dep} = 1;
			}
		}
	}

	for my $i (0 .. @$parameters - 1)
	{
		$params[$i]{name} = $parameters->[$i]{name} . "Arg";

		if (not $params[$i]{type})
		{
			my ($t, $deps) = haskell_type($parameters->[$i]{type});

			$params[$i]{type} = $t;

			foreach my $dep (@$deps)
			{
				$dependencies{$dep} = 1;
			}
		}
	}

	# validate the result
	for my $i (0 .. $#params)
	{
		die "Could not translate parameters" unless ($params[$i]{name} and $params[$i]{type});
	}

	return (\@params, [keys %dependencies]);
}


sub haskell_type
{
	my ($type) = @_;

	if ($type =~ /^int(32|64)/)
	{
		return ("Data.Int.Int$1", ["Data.Int"]);
	}
	elsif ($type =~ /^uint(32|64)/)
	{
		return ("Data.Word.Word$1", ["Data.Word"]);
	}
	elsif (lc $type eq 'double')
	{
		return ("Double", []);
	}
	elsif (lc $type eq 'float')
	{
		return ("Float", []);
	}
	elsif (lc $type eq 'bool')
	{
		return ("Bool", []);
	}
	elsif (lc $type eq 'string')
	{
		return ("Data.Text.Text", ["Data.Text"]);
	}
	elsif ($type =~ /^(Class|Enum)\((.+)\)$/)
	{
		my ($obj, $dep) = reverse split(/\./, $2, 2);
		return ("KRPCHS.$dep.$obj", ["KRPCHS.$dep"]);
	}
	elsif ($type =~ /^List\((.+)\)$/)
	{
		my ($obj, $dep) = haskell_type($1);
		return ("[$obj]", $dep);
	}
	elsif ($type =~ /^Tuple\((.+)\)$/)
	{
		my @objs;
		my @deps;
		
		foreach my $e (split /,/, $1)
		{
			my ($obj, $dep) = haskell_type($e);
			push @objs, $obj;
			push @deps, @$dep;
		}

		return ("(" . join(", ", @objs) . ")", \@deps);
	}
	elsif ($type =~ /^Dictionary\((.+),(.+)\)/)
	{
		my ($keyType, $keyDeps) = haskell_type($1);
		my ($valType, $valDeps) = haskell_type($2);
		my %deps = map { $_ => 1 } ("Data.Map", @$keyDeps, @$valDeps);
		return ("Data.Map.Map ($keyType) ($valType)", [ keys %deps ]);
	}
	else
	{
		die "Unknown type '$type'"
	}
}


sub cleanup_doc
{
	my ($doc) = @_;
	$doc =~ s@(\s+)?</?\w+/?>(\s+)?@@gr
}


__DATA__
module KRPCHS.[% serviceName %]
( [% exports.join("\n, ") %]
) where

[%- FOREACH i IN imports %]
import qualified [% i %]
[%- END %]

import KRPCHS.Internal.Requests
import KRPCHS.Internal.SerializeUtils


[%- FOREACH proxy IN proxys %]
[%- accessor = proxy.name.lcfirst _ 'Id' %]
{-
[% proxy.doc.replace('{-', '{ -') FILTER format(' - %s') %]
 -}
newtype [% proxy.name %] = [% proxy.name %] { [% accessor %] :: Int }
    deriving (Show, Eq, Ord)

instance PbSerializable [% proxy.name %] where
    encodePb   = encodePb . [% accessor %]
    decodePb b = [% proxy.name %] <$> decodePb b

instance KRPCResponseExtractable [% proxy.name %]

[%- END %]

[%- FOREACH enum IN enums %]
{-
[% enum.doc.replace('{-', '{ -') FILTER format(' - %s') %]
 -}
data [% enum.name %]
    = [% enum.values.join("\n    | ") %]
    deriving (Show, Eq, Ord, Enum)

instance PbSerializable [% enum.name %] where
    encodePb   = encodePb . fromEnum
    decodePb b = toEnum <$> decodePb b

instance KRPCResponseExtractable [% enum.name %]

[%- END %]

[%- FOREACH proc IN procs %]
[%- types = [] %]
[%- names = [] %]
[%- args  = [] %]
[%- FOREACH p IN proc.params %]
[%- types.push(p.type) %]
[%- names.push(p.name) %]
[%- args.push('makeArgument ' _ (loop.count - 1) _ ' ' _ p.name) %]
[%- END %]
{-
[% proc.doc.replace('{-', '{ -') FILTER format(' - %s') %]
 -}
[%- ret = proc.ret || 'Bool' %]
[%- IF types.size > 0 %]
[% proc.name %] :: [% types.join(' -> ') %] -> RPCContext ([% ret %])
[%- ELSE %]
[% proc.name %] :: RPCContext ([% ret %])
[%- END %]
[% proc.name %] [% names.join(' ') %] = do
    let r = makeRequest "[% serviceName %]" "[% proc.rpcname %]" [[% args.join(', ') %]]
    res <- sendRequest r
    [%- IF proc.ret %]
    processResponse extract res
    [%- ELSE %]
    processResponse extractNothing res
    [% END %] 
[%- IF proc.ret %]

[%- IF types.size > 0 %]
[% proc.name _ 'Stream' %] :: [% types.join(' -> ') %] -> RPCContext (KRPCStream ([% proc.ret %]))
[%- ELSE %]
[% proc.name _ 'Stream' %] :: RPCContext (KRPCStream ([% proc.ret %]))
[%- END %]
[% proc.name _ 'Stream' %] [% names.join(' ') %] = do
    let r = makeRequest "[% serviceName %]" "[% proc.rpcname %]" [[% args.join(', ') %]]
        s = makeStream r
    res <- sendRequest s
    sid <- processResponse extract res
    return $ KRPCStream sid
[%- END %] 


[%- END %]
