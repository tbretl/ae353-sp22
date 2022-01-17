from json import load, dump
from argparse import ArgumentParser

# This code was inspired by Nytko and Silva (CS 357).

def main():
    parser = ArgumentParser()
    parser.add_argument('infile', metavar='infile.ipynb')
    parser.add_argument('outfile', metavar="outfile.ipynb")
    args = parser.parse_args()

    with open(args.infile, 'rt', encoding='utf-8') as infile:
        ipynb = load(infile)

    ipynb['metadata'] = {
        'kernelspec': {
            'display_name': 'Python 3',
            'langage': 'python',
            'name': 'python3',
        },
        'language_info': {
            'codemirror_mode': {
                'name': 'ipython',
                'version': 3,
            },
            'file_extension': '.py',
            'mimetype': 'text/x-python',
            'name': 'python',
            'nbconvert_exporter': 'python',
            'pygments_lexer': 'ipython3',
        }
    }

    if 'worksheets' in ipynb:
        raise Exception('found "worksheets" in the notebook...')

    for cell in ipynb['cells']:
        if cell['cell_type'] == 'code':
            cell['outputs'] = []
            cell['execution_count'] = None

    with open(args.outfile, 'wt') as outfile:
        dump(ipynb, outfile, indent=1, sort_keys=True)

if __name__ == '__main__':
    main()
