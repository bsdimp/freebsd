# FreeBSD License Policy

SPDX is a standard way of indicating a license for files in an open source project's tree.

The FreeBSD project's policy for contribution is that all files that are contributed must contain a license which grants use of the file under acceptable terms.
The FreeBSD core team decides which licenses are similar to our preferred license (the 2 clause BSD license).
All files contributed either must contain a verbatim copy of the license they are contributed under, or they must contain a line that has SPDX-License-Identifier on it.
For files that contain both, the SPDX-License-Identifier line is informative and the verbatim licenses are nomitive.

The FreeBSD project currently followed the SPDX specification (version 2.2) as articulated in Appendix IV and Appendix V.

Licenses in the SPDX-License-Identifier lines are represented by a short identifier from the following list.
The text for these licenses will be contained in the share/license/text directory in the FreeBSD src repository with a file of the same name as the short identifier with ".txt" appended to the end.
Only those short identifiers appearing below may have their license text omitted in the FreeBSD source tree.
While other identifiers may appear in informative SPDX-License-Identifier line expressions, only these are permitted when the license text is omitted.
"GPL-2.0+" is an alias for "GPL-2.0-or-later".
Multiple licenses, and modifications to licenses may be specified using a SPDX License Expression, as defined in Appendix IV, Compisite License Expression section, reproduced here.

In the event that a short identifier appears below, but a file of identical name does not appear in the share/license directory, that license text may not be omitted from contributed text.
Please file a problem report at https://bugs.freebsd.org/ to apprise of the issue.

## Composite License Expressions

Note: This section reproduced from Appendix IV of the SPDX specification, v2.2.0 Copyright (c) 2010-2020 Linux Foundation and its Contributors under the SPDX license identifier CC-BY-3.0.

More expressive composite license expressions can be constructed using "OR", "AND", and "WITH" operators similar to constructing mathematical expressions using arithmetic operators.

For the tag:value format, any license expression that consists of more than one license identifier and/or LicenseRef, may optionally be encapsulated by parentheses: "( )".

Nested parentheses can also be used to specify an order of precedence which is discussed in more detail in subsection (4).

### Disjunctive "OR" Operator
If presented with a choice between two or more licenses, use the disjunctive binary "OR" operator to construct a new license expression, where both the left and right operands are valid license expression values.

For example, when given a choice between the LGPL-2.1-only or MIT licenses, a valid expression would be:

```
LGPL-2.1-only OR MIT
```
An example representing a choice between three different licenses would be:

```
LGPL-2.1-only OR MIT OR BSD-3-Clause
```
### Conjunctive "AND" Operator
If required to simultaneously comply with two or more licenses, use the conjunctive binary "AND" operator to construct a new license expression, where both the left and right operands are a valid license expression values.

For example, when one is required to comply with both the LGPL-2.1-only or MIT licenses, a valid expression would be:

```
LGPL-2.1-only AND MIT
```
An example where all three different licenses apply would be:

```
LGPL-2.1-only AND MIT AND BSD-2-Clause
```

### Exception "WITH" Operator
Sometimes a set of license terms apply except under special circumstances. In this case, use the binary "WITH" operator to construct a new license expression to represent the special exception situation.
A valid <license-expression> is where the left operand is a <simple-expression> value and the right operand is a <license-exception-id> that represents the special exception terms.

For example, when the Bison exception is to be applied to GPL-2.0-or-later, the expression would be:

```
GPL-2.0-or-later WITH Bison-exception-2.2
```
The current set of valid exceptions can be found in the Acceptable Exceptions subsection.
If the applicable exception is not found on the SPDX License Exception List, then the entire license must be included in the file.

### Order of Precedence and Parentheses

The order of application of the operators in an expression matters (similar to mathematical operators). The default operator order of precedence of a <license-expression> a is:

```
+
WITH
AND
OR
```
where a lower order operator is applied before a higher order operator.

For example, the following expression:

```
LGPL-2.1-only OR BSD-3-Clause AND MIT
```
represents a license choice between either LGPL-2.1-only and the expression BSD-3-Clause AND MIT because the AND operator takes precedence over (is applied before) the OR operator.

When required to express an order of precedence that is different from the default order a <license-expression> can be encapsulated in pairs of parentheses: ( ), to indicate that the operators found inside the parentheses takes precedence over operators outside.
This is also similar to the use of parentheses in an algebraic expression e.g., (5+7)/2.

For instance, the following expression:

```
MIT AND (LGPL-2.1-or-later OR BSD-3-Clause)
```
states the OR operator should be applied before the AND operator.
That is, one should first select between the LGPL-2.1-or-later or the BSD-3-Clause license before applying the MIT license.

## Acceptable Licenses

```
BSD-1-Clause
BSD-2-Clause
BSD-2-Clause-Patent
BSD-3-Clause
BSD-4-Clause
GPL-2.0-only
GPL-2.0-or-later
ISC
MIT
MIT-CMU
RSA-MD
X11
```

## Acceptable Exceptions
none at this time
