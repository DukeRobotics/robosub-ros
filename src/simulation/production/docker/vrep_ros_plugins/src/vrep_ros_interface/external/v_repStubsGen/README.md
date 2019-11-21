# V-REP Stubs generator

This utility is used to generate boilerplate code for V-REP Lua callbacks.
It reads an XML file containing a description of the callbacks, script
functions, and enums, and it produces a pair of C++ source/header files.

## Dependencies

- Python interpreter (2.7 or greater)
- xsltproc program in your PATH (an XSLT processor) if you want to generate documentation (windows binary available [here](https://github.com/CoppeliaRobotics/xsltproc-win/raw/master/xsltproc-win.zip))

## Usage

```text
$ python generate.py --help
usage: generate.py [-h] [--xml-file XML_FILE] [--lua-file LUA_FILE]
                   [--gen-stubs] [--gen-lua-xml] [--gen-reference-xml]
                   [--gen-reference-html] [--gen-lua-calltips]
                   [--gen-notepadplusplus-stuff] [--gen-deprecated-txt]
                   [--gen-all]
                   output_dir

Generate various things for V-REP plugin.

positional arguments:
  output_dir            the output directory

optional arguments:
  -h, --help            show this help message and exit
  --xml-file XML_FILE   the XML file with the callback definitions
  --lua-file LUA_FILE   an optional LUA file containing docstrings
  --gen-stubs           generate C++ stubs
  --gen-lua-xml         generate XML translation of Lua docstrings
  --gen-reference-xml   generate merged XML (from callbacks.xml and lua.xml)
  --gen-reference-html  generate HTML documentation (from reference.xml or
                        callbacks.xml)
  --gen-lua-calltips    generate C++ code for Lua calltips
  --gen-notepadplusplus-stuff
                        generate syntax hilighting stuff for notepad++
  --gen-deprecated-txt  generate deprecated functions mapping for V-REP
  --gen-all             generate everything
```

## Example

The `callbacks.xml` files has this structure:

```xml
<plugin name="PluginName" short-name="PLG" author="you@example.com">
    <description>An example plugin to do a useless thing</description>

    <!--
        put <command>, <script-function>, <enum> or <struct> elements here
    -->
</plugin>
```

### Commands and Script Functions

Each `<command>` or `<script-function>` element serves both as a declaration and documentation for a Lua command/callback of the plugin. It can contain a description of the command itself, and several input and output parameters, with annotated type and documentation.

Example:

```xml
    <command name="test">
        <description>Do something not much useful.</description>
        <params>
            <param name="a" type="int">
                <description>An integer argument</description>
            </param>
            <param name="b" type="string">
                <description>An string argument</description>
            </param>
            <param name="c" type="table" item-type="float">
                <description>A vector of floats</description>
            </param>
        </params>
        <return>
            <param name="x" type="float">
                <description>A float return value</description>
            </param>
            <param name="y" type="string">
                <description>A string return value</description>
            </param>
        </return>
    </command>
```

The tool will generate C++ code:
 - a `test_in` data structure for the input arguments
 - a `test_out` data structure for the output arguments

In C++ code, the corresponding callback must be implemented:

```C++
void test(SScriptCallBack *p, const char *cmd, test_in *in, test_out *out)
{
    ...
}
```

### Data Structures

The `<struct>` element will generate a C++ struct, and documentation for the data structure.

Example:

```xml
    <struct name="mystruct">
        <description>Extra options for the curve creation function <command-ref name="addCurve"/>.</description>
        <param name="a" type="int" default="1">
            <description>an integer field</description>
        </param>
        <param name="b" type="bool" default="true">
            <description>a boolean field</description>
        </param>
    </struct>
```

Then in C++ it can be used like this:

```C++
mystruct x;
x.a = 3;
x.b = false;
```

or passed as parameter to commands.

### Enumerations

The `<enum>` element will generate a C++ enum, and documentation for that.

Example:

```xml
    <enum name="myenum" item-prefix="foo_" base="32000">
        <item name="alpha" />
        <item name="beta" />
        <item name="gamma" />
    </enum>
```

It will generate an enum named `myenum` with items `sim_plugname_foo_alpha`, `sim_plugname_foo_beta`, `sim_plugname_foo_gamma`.

## Complete example

See [v_repExtPluginSkeletonNG](https://github.com/CoppeliaRobotics/v_repExtPluginSkeletonNG) for an example of a V-REP plugin using this framework.

