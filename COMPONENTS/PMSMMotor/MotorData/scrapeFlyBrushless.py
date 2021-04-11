# %%
from bs4 import BeautifulSoup
import requests
import re
import time
import json
# %%
main_index_url = "http://www.flybrushless.com/search/advance/type/outrunner/"
main_index_page = requests.get(main_index_url)
main_index_bs = BeautifulSoup(main_index_page.content, 'html.parser')

# Find Number of Pages
div_brousebypage = main_index_bs.find('div', class_='browsebypage')
last_page_elem = div_brousebypage.find('a', text='Last ›')
last_page_url = last_page_elem['href']
last_page_N =  int(re.findall(r'\d+', last_page_url)[0])

# Get list of page urls to parse for links
page_numbers = [20*x for x in range(int(last_page_N/20)+1)]
index_page_urls = [main_index_url + "page/" + str(n) for n in page_numbers]

# %%
# Parse each index page for motor pages
motor_links = []
for url in index_page_urls:
    page = requests.get(url)
    bs = BeautifulSoup(page.content, 'html.parser')
    def motor_link_filter(tag):
        par = tag.parent
        c1 = par.name == 'div'
        if par.has_attr('class'):
            c2 = (par['class'] == ['browsebycell1']) or (par['class'] == ['browsebycell2'])
        else:
            c2 = False
        c3 = tag.name == 'a'
        c4 = tag.has_attr('href')
        return all([c1,c2,c3,c4])
    motor_elems = bs.find_all(motor_link_filter)
    motor_links.extend([x['href'] for x in motor_elems])


# %%
motor_list = []
for link in motor_links:
    page = requests.get(link)
    bs = BeautifulSoup(page.content, 'html.parser')
    main = bs.find('div', id='main')
    specs_elem = main.find('div', class_ = 'motorspecs')
    constant_elem = main.find('div', class_ = 'motorconstant')

    motor_dict = {}
    [make,model] = main.find('h1').string.split(' - ',1)
    motor_dict['MAKE'] = make
    motor_dict['MODEL'] = model

    specs_dict = {}
    specs_elems = specs_elem.find_all('li')
    for s in specs_elems:
        spec_str = s.string
        [key,val] = spec_str.split(': ',1)
        key = key.replace(' ', '_')

        # Parse val for numbers
        nums = re.findall(r'\d+\.?\d*', val)
        if nums:
            num = nums[0] # Only need first number
            unit = re.findall(r'(?<='+num+r')\w+', val)
            if unit:
                unit = unit[0]
            else: 
                unit = ''

            val = {"Value":float(num), "Unit":unit}

        specs_dict[key] = val

    motor_dict['SPECS'] = specs_dict

    constant_dict = {}
    constant_elems = constant_elem.find_all('li')
    for s in constant_elems:
        spec_str = s.string
        [key,val] = spec_str.split(': ',1)
        key = key.replace(' ', '_')

        # Parse val for numbers
        nums = re.findall(r'\d+\.?\d*', val)
        if nums:
            num = nums[0] # Only need first number
            unit = re.findall(r'(?<='+num+r')\w+', val)
            if unit:
                unit = unit[0]
            else: 
                unit = ''

            val = {"Value":float(num), "Unit":unit}

        constant_dict[key] = val

    motor_dict['CONSTANTS'] = constant_dict
    print(motor_dict)
    motor_list.append(motor_dict)
    time.sleep(0.25)

with open('motor_database.json','w') as f:
    json.dump(motor_list, f, indent=2)
# %%
