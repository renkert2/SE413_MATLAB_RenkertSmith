# %%
from bs4 import BeautifulSoup
import requests
import re
import time
import json
# %%
main_index_url = "https://www.kdedirect.com/collections/uas-multi-rotor-brushless-motors"
main_index_page = requests.get(main_index_url)
main_index_bs = BeautifulSoup(main_index_page.content, 'html.parser')

# %%
# Parse each index page for motor pages
motor_grid = main_index_bs.find('div', class_='grid-uniform')
motor_elems = motor_grid.find_all('a', class_='product-grid-item')

motor_links = ['https://www.kdedirect.com'+x['href'] for x in motor_elems]
# %%
motor_list = []
for link in motor_links:
    page = requests.get(link)
    bs = BeautifulSoup(page.content, 'html.parser')
    motor_dict = {}

    motor_dict['MAKE'] = 'KDE'

    model = bs.find('span', id='product-sku', itemprop='sku').text
    motor_dict['MODEL'] = model

    price = bs.find('meta', itemprop='price')['content']
    price = price.replace(',','')
    price = float(re.findall(r'\d+\.?\d*', price)[0])

    motor_dict['PRICE'] = price

    spec_table = bs.find('tbody')
    specs = {}
    for spec in spec_table.find_all('tr'):
        spec_elems = spec.find_all('td')
        desc = spec_elems[0].text
        desc = re.sub("\(.*?\)", "", desc).strip()
        desc = desc.replace(' ', '_').replace('*','')

        val = spec_elems[1].text
        val = val.replace(',','').replace('Ω','Ohm').replace('ф','').replace('°','deg').replace('√(W)','sqrt[W]').strip()

        # Get units from val
        num_units = re.findall(r'(\d+\.?\d*) ([^\d\s\(\)]*)', val)
        if len(num_units) > 1:
            val = re.sub("\(.*?\)", "", val).strip()
            num_units = re.findall(r'(\d+\.?\d*) ([^\d\s\(\)]*)', val)
        if len(num_units) == 1:
            num_unit = num_units[0]
            num = num_unit[0]
            unit = num_unit[1]
            if not unit:
                unit = ''

            val = {"Value":float(num), "Unit":unit}

        specs[desc] = val

    motor_dict['SPECS'] = specs

    print(motor_dict)
    motor_list.append(motor_dict)
    time.sleep(0.25)

# %%
with open('motor_database_kde.json','w') as f:
    json.dump(motor_list, f, indent=2)
# %%
