"""
Launch substitution processing a xacro file: $(xacro-file <file> [name:=value ...]).

The result is the processed document, as `xacro <file> name:=value ...` prints it, e.g. for the robot_description
parameter of robot_state_publisher (type: str). Compared to $(command 'xacro ...'), the file is processed in the
launch process: no shell is involved, values with spaces only need launch quotes, and xacro errors are reported as
substitution failures with the xacro message.
"""

import xml.parsers.expat
from typing import Dict, List, Sequence, Text, Tuple

from launch.frontend import expose_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.substitutions import SubstitutionFailure
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

import xacro


def parse_mapping(text: Text) -> Tuple[Text, Text]:
    """Return name and value of a name:=value argument."""
    name, separator, value = text.partition(':=')
    name = name.strip()
    if not separator or not name:
        raise SubstitutionFailure(f"xacro-file: argument '{text}' is not name:=value")
    return name, value.strip()


def process(path: Text, mappings: Dict[Text, Text]) -> Text:
    """Return the processed xacro file as XML text."""
    try:
        return xacro.process_file(path, mappings=mappings).toprettyxml(indent='  ')
    except xml.parsers.expat.ExpatError as e:
        raise SubstitutionFailure(f'xacro-file: {path}: XML parsing error: {e}')
    except Exception as e:  # evaluated expressions can raise anything
        raise SubstitutionFailure(f'xacro-file: {path}: {e or repr(e)}')


@expose_substitution('xacro-file')
class XacroFile(Substitution):
    """Substitution processing a xacro file with name:=value arguments."""

    def __init__(self, path: SomeSubstitutionsType, mappings: Sequence[SomeSubstitutionsType] = ()) -> None:
        """
        Create a XacroFile substitution.

        :param path: the xacro file
        :param mappings: the arguments of the file, each 'name:=value'
        """
        super().__init__()
        self.__path = normalize_to_list_of_substitutions(path)
        self.__mappings = [normalize_to_list_of_substitutions(m) for m in mappings]

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        """Parse $(xacro-file <file> [name:=value ...])."""
        if not data:
            raise TypeError('xacro-file substitution expects <file> [name:=value ...]')
        return cls, {'path': data[0], 'mappings': data[1:]}

    @property
    def path(self) -> List[Substitution]:
        """Getter for the file."""
        return self.__path

    @property
    def mappings(self) -> List[List[Substitution]]:
        """Getter for the arguments."""
        return self.__mappings

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        parts = [self.__path] + self.__mappings
        return 'XacroFile({})'.format(', '.join(' + '.join(s.describe() for s in part) for part in parts))

    def perform(self, context: LaunchContext) -> Text:
        """Return the processed xacro file."""
        path = perform_substitutions(context, self.__path)
        mappings = dict(parse_mapping(perform_substitutions(context, m)) for m in self.__mappings)
        return process(path, mappings)
