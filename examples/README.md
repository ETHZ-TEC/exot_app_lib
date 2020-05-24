The file *[example.json](example.json)* contains an sample config file used by a JSON-configured application.

- It includes the complete settings of all currently available configurable components.
- The values in the JSON file map to settings structures containing native data types.
- Appropriate value types should be used. Type mismatch will result in an exception, which should be descriptive enough to help correct the error.
- Specific JSON←→native converters may throw exceptions if values are malformed or incorrect. The exceptions should be fairly informative. This may commonly happen for values that map to C++ enum classes, allowing only a range of possible string values.
- To use defaults defined in the C++ files either remove a key-value pair from the config, or set it to null.

For further details please review the header files to determine data types, or consult the wiki.[^1]

---

[^1]: Coming soon.
