#!/usr/bin/env perl

use strict;
use warnings;

use Carp;
use JSON;
use Template;
use Data::Dumper;


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
		push @exports, $proc->{name} . 'Stream'    if ($proc->{ret});
		push @exports, $proc->{name} . 'StreamReq' if ($proc->{ret});

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
		[ sort keys %deps ]
	);
}


sub process_return_type
{
	my ($proc) = @_;
	return haskell_type($proc->{return_type});
}


sub process_parameters
{
	my ($proc) = @_;

	my @params;
	my @dependencies;

	foreach my $p (@{$proc->{parameters}})
	{
		my ($t, $d) = haskell_type($p->{type});
		push @params, { name => lcfirst $p->{name} . "Arg", type => $t };
		push @dependencies, @$d;
	}

	return (\@params, \@dependencies);
}


sub haskell_type
{
	my ($type) = @_;

	if (!$type)
	{
		return ("", []);
	}

	elsif (!$type->{code})
	{
		confess "Can't find this type's code" . Dumper $type;
	}

	elsif ($type->{code} eq 'CLASS' or $type->{code} eq 'ENUMERATION')
	{
		my $dep = "KRPCHS.$type->{service}";
		my $obj = "KRPCHS.$type->{service}.$type->{name}";
		return ($obj, [$dep]);
	}

	elsif ($type->{code} eq 'LIST')
	{
		my ($content, $deps) = haskell_type($type->{types}[0]);
		return ("[$content]", $deps);
	}

	elsif ($type->{code} eq 'TUPLE')
	{
		my @deps;
		my @elems;

		foreach my $e (@{$type->{types}})
		{
			my ($t, $d) = haskell_type($e);
			push @deps, @$d;
			push @elems, $t;
		}

		return ("(" . join(", ", @elems) . ")", \@deps);
	}

	elsif ($type->{code} eq 'DICTIONARY')
	{
		my ($key_type, $key_deps) = haskell_type($type->{types}[0]);
		my ($val_type, $val_deps) = haskell_type($type->{types}[1]);
		return
			( "Data.Map.Map ($key_type) ($val_type)"
			, ["Data.Map", @$key_deps, @$val_deps]
		    )
	}

	elsif ($type->{code} eq 'STRING')
	{
		return ("Data.Text.Text", ["Data.Text"]);
	}

	elsif ($type->{code} eq 'BOOL')
	{
		return ("Bool", []);
	}

	elsif ($type->{code} eq 'DOUBLE')
	{
		return ("Double", []);
	}

	elsif ($type->{code} eq 'FLOAT')
	{
		return ("Float", []);
	}

	elsif ($type->{code} =~ /SINT(32|64)/)
	{
		return ("Data.Int.Int$1", ["Data.Int"]);
	}

	elsif ($type->{code} =~ /UINT(32|64)/)
	{
		return ("Data.Word.Word$1", ["Data.Word"]);
	}

	else
	{
		confess "Unknown type '$type'"
	}
}


sub cleanup_doc
{
	my ($doc) = @_;
	utf8::encode($doc);                 # some bad chars
	$doc =~ s@(\s+)?</?\w+/?>(\s+)?@@gr # remove most xml bits
}


__DATA__
module KRPCHS.[% serviceName %]
( [% exports.join("\n, ") %]
) where

[%- FOREACH i IN imports %]
import qualified [% i %]
[%- END %]

import KRPCHS.Internal.Requests
import KRPCHS.Internal.Requests.Call
import KRPCHS.Internal.Requests.Stream
import KRPCHS.Internal.SerializeUtils


[%- FOREACH proxy IN proxys %]
[%- accessor = proxy.name.lcfirst _ 'Id' %]
{-|
[% proxy.doc.replace('{-', '{ -') %]
 -}
newtype [% proxy.name %] = [% proxy.name %] { [% accessor %] :: Int }
    deriving (Show, Eq, Ord)

instance PbEncodable [% proxy.name %] where
    encodePb = encodePb . [% accessor %]

instance PbDecodable [% proxy.name %] where
    decodePb b = [% proxy.name %] <$> decodePb b

instance KRPCResponseExtractable [% proxy.name %]

[%- END %]

[%- FOREACH enum IN enums %]
{-|
[% enum.doc.replace('{-', '{ -') %]
 -}
data [% enum.name %]
    = [% enum.values.join("\n    | ") %]
    deriving (Show, Eq, Ord, Enum)

instance PbEncodable [% enum.name %] where
    encodePb = encodePb . fromEnum

instance PbDecodable [% enum.name %] where
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
{-|
[% proc.doc.replace('{-', '{ -') %]
 -}
[%- IF types.size > 0 %]
[% proc.name %] :: [% types.join(' -> ') %] -> RPCContext ([% proc.ret %])
[%- ELSE %]
[% proc.name %] :: RPCContext ([% proc.ret %])
[%- END %]
[% proc.name %] [% names.join(' ') %] = do
    let r = makeRequest "[% serviceName %]" "[% proc.rpcname %]" [[% args.join(', ') %]]
    res <- sendRequest r
    processResponse res
[%- IF proc.ret %]

[%- IF types.size > 0 %]
[% proc.name _ 'StreamReq' %] :: [% types.join(' -> ') %] -> KRPCStreamReq ([% proc.ret %])
[%- ELSE %]
[% proc.name _ 'StreamReq' %] :: KRPCStreamReq ([% proc.ret %])
[%- END %]
[% proc.name _ 'StreamReq' %] [% names.join(' ') %] =
    let req = makeCallRequest "[% serviceName %]" "[% proc.rpcname %]" [[% args.join(', ') %]]
    in  makeStream req

[%- IF types.size > 0 %]
[% proc.name _ 'Stream' %] :: [% types.join(' -> ') %] -> RPCContext (KRPCStream ([% proc.ret %]))
[%- ELSE %]
[% proc.name _ 'Stream' %] :: RPCContext (KRPCStream ([% proc.ret %]))
[%- END %]
[% proc.name _ 'Stream' %] [% names.join(' ') %] = requestStream $ [% proc.name _ 'StreamReq' %] [% names.join(' ') %]
[%- END %] 

[%- END %]
